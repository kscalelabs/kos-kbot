use async_trait::async_trait;
use chrono::Local;
use eyre::{eyre, Result, WrapErr};
use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app as gst_app;
use kos::{
    hal::{KClipStartResponse, KClipStopResponse, ProcessManager},
    kos_proto::common::{Error, ErrorCode},
    services::TelemetryLogger,
    telemetry::Telemetry,
};
use krec::combine_with_video;
use std::env;
use std::path::{Path, PathBuf};
use tokio::sync::Mutex;
use uuid::Uuid;

pub struct KBotProcessManager {
    kclip_uuid: Mutex<Option<String>>,
    pipeline: Mutex<Option<gst::Pipeline>>,
    telemetry_logger: Mutex<Option<TelemetryLogger>>,
    current_action: Mutex<Option<String>>,
    robot_name: String,
    robot_serial: String,
}

impl KBotProcessManager {
    pub fn new(robot_name: String, robot_serial: String) -> Result<Self> {
        gst::init().wrap_err("Failed to initialize GStreamer")?;

        Ok(KBotProcessManager {
            kclip_uuid: Mutex::new(None),
            pipeline: Mutex::new(None),
            telemetry_logger: Mutex::new(None),
            current_action: Mutex::new(None),
            robot_name,
            robot_serial,
        })
    }

    fn create_pipeline(video_path: &Path) -> Result<(gst::Pipeline, gst::Element)> {
        let pipeline = gst::Pipeline::new(None);

        // Create elements
        let src = gst::ElementFactory::make("v4l2src")
            .name("src")
            .property("device", "/dev/video47")
            .property_from_str("io-mode", "2") // Force memory mapping mode
            .build()
            .wrap_err("Failed to create v4l2src")?;

        let src_caps = gst::ElementFactory::make("capsfilter")
            .name("src_caps")
            .property(
                "caps",
                gst::Caps::builder("video/x-raw")
                    .field("format", "YUY2")
                    .field("width", 1280i32)
                    .field("height", 1080i32)
                    .field("framerate", gst::Fraction::new(30, 1))
                    .build(),
            )
            .build()
            .wrap_err("Failed to create source capsfilter")?;

        let videoscale = gst::ElementFactory::make("videoscale")
            .name("videoscale0")
            .build()
            .wrap_err("Failed to create videoscale")?;

        let videoflip = gst::ElementFactory::make("videoflip")
            .name("videoflip0")
            .property_from_str("video-direction", "vert")
            .build()
            .wrap_err("Failed to create videoflip")?;

        let scale_caps = gst::ElementFactory::make("capsfilter")
            .name("scale_caps")
            .property(
                "caps",
                gst::Caps::builder("video/x-raw")
                    .field("width", 512i32)
                    .field("height", 512i32)
                    .build(),
            )
            .build()
            .wrap_err("Failed to create scale capsfilter")?;

        let tee = gst::ElementFactory::make("tee")
            .name("tee0")
            .build()
            .wrap_err("Failed to create tee")?;

        let queue_monitor = gst::ElementFactory::make("queue")
            .name("queue_monitor")
            .build()
            .wrap_err("Failed to create monitor queue")?;

        let queue_record = gst::ElementFactory::make("queue")
            .name("queue_record")
            .build()
            .wrap_err("Failed to create record queue")?;

        let videoconvert_monitor = gst::ElementFactory::make("videoconvert")
            .name("videoconvert_monitor")
            .build()
            .wrap_err("Failed to create monitor videoconvert")?;

        let videoconvert_record = gst::ElementFactory::make("videoconvert")
            .name("videoconvert_record")
            .build()
            .wrap_err("Failed to create record videoconvert")?;

        let appsink = gst_app::AppSink::builder()
            .name("appsink0")
            .caps(
                &gst::Caps::builder("video/x-raw")
                    .field("format", "RGB")
                    .build(),
            )
            .build();

        appsink.set_callbacks(
            gst_app::AppSinkCallbacks::builder()
                .new_sample(move |appsink| {
                    let sample = appsink.pull_sample().map_err(|_| gst::FlowError::Error)?;
                    let buffer = sample.buffer().ok_or(gst::FlowError::Error)?;

                    let pts = buffer.pts();
                    let telemetry = Telemetry::try_get();
                    if let Some(telemetry) = telemetry {
                        if let Some(pts) = pts {
                            telemetry.update_video_timestamp(pts.nseconds());
                            telemetry.increment_frame_number();
                        }
                    }

                    Ok(gst::FlowSuccess::Ok)
                })
                .build(),
        );

        let encoder = gst::ElementFactory::make("x264enc")
            .name("x264enc0")
            .property_from_str("tune", "stillimage")
            .property_from_str("speed-preset", "ultrafast")
            .build()
            .wrap_err("Failed to create H264 encoder")?;

        let parser = gst::ElementFactory::make("h264parse")
            .name("h264parse0")
            .build()
            .wrap_err("Failed to create H264 parser")?;

        let muxer = gst::ElementFactory::make("qtmux").name("qtmux0").build()?;

        let sink = gst::ElementFactory::make("filesink")
            .name("filesink0")
            .property(
                "location",
                video_path
                    .to_str()
                    .ok_or_else(|| eyre!("Invalid video path"))?,
            )
            .build()
            .wrap_err("Failed to create filesink")?;

        // Add elements to pipeline
        pipeline
            .add_many(&[
                &src,
                &src_caps,
                &videoscale,
                &videoflip,
                &scale_caps,
                &tee,
                &queue_monitor,
                &videoconvert_monitor,
                &queue_record,
                &videoconvert_record,
                appsink.upcast_ref(),
                &encoder,
                &parser,
                &muxer,
                &sink,
            ])
            .wrap_err("Failed to add elements to pipeline")?;

        // Link elements up to tee
        gst::Element::link_many(&[&src, &src_caps, &videoscale, &videoflip, &scale_caps, &tee])?;

        // Link monitoring branch
        gst::Element::link_many(&[&queue_monitor, &videoconvert_monitor, appsink.upcast_ref()])?;

        // Link recording branch
        gst::Element::link_many(&[
            &queue_record,
            &videoconvert_record,
            &encoder,
            &parser,
            &muxer,
            &sink,
        ])?;

        // Link tee to both queues
        tee.link_pads(Some("src_0"), &queue_monitor, None)?;
        tee.link_pads(Some("src_1"), &queue_record, None)?;

        Ok((pipeline, appsink.upcast()))
    }

    // Add a method to get the recordings directory
    fn recordings_dir() -> PathBuf {
        // Use XDG_DATA_HOME if set, otherwise default to ~/.local/share/kos/recordings
        env::var_os("XDG_DATA_HOME")
            .map(PathBuf::from)
            .unwrap_or_else(|| {
                let home = env::var_os("HOME")
                    .map(PathBuf::from)
                    .unwrap_or_else(|| PathBuf::from("/tmp"));
                home.join(".local/share")
            })
            .join("kos/recordings")
    }

    // Add a method to ensure the recordings directory exists
    fn ensure_recordings_dir() -> Result<PathBuf> {
        let dir = Self::recordings_dir();
        std::fs::create_dir_all(&dir).wrap_err_with(|| {
            format!("Failed to create recordings directory: {}", dir.display())
        })?;
        Ok(dir)
    }

    // Add a method to generate paths for a recording
    fn recording_paths(uuid: &str, action: &str) -> Result<(PathBuf, PathBuf, PathBuf)> {
        let dir = Self::ensure_recordings_dir()?;
        let timestamp = Local::now().format("%Y%m%d_%H%M%S");
        let safe_action = action.replace(|c: char| !c.is_alphanumeric(), "_");
        Ok((
            dir.join(format!("telemetry_{}_{}.krec", safe_action, uuid)),
            dir.join(format!("video_{}_{}.mkv", safe_action, uuid)),
            dir.join(format!(
                "recording_{}_{}_{}.krec.mkv",
                timestamp, safe_action, uuid
            )),
        ))
    }
}

#[async_trait]
impl ProcessManager for KBotProcessManager {
    async fn start_kclip(&self, action: String) -> Result<KClipStartResponse> {
        let mut kclip_uuid = self.kclip_uuid.lock().await;
        if kclip_uuid.is_some() {
            return Ok(KClipStartResponse {
                clip_uuid: None,
                error: Some(Error {
                    code: ErrorCode::InvalidArgument as i32,
                    message: "KClip is already started".to_string(),
                }),
            });
        }

        *self.current_action.lock().await = Some(action.clone());

        let new_uuid = Uuid::new_v4().to_string();
        let (telemetry_path, video_path, _) = Self::recording_paths(&new_uuid, &action)?;

        *kclip_uuid = Some(new_uuid.clone());
        drop(kclip_uuid);

        let (pipeline, _sink) = Self::create_pipeline(&video_path)?;

        // Start the pipeline
        pipeline.set_state(gst::State::Playing)?;

        // Start telemetry logger
        let logger = TelemetryLogger::new(
            new_uuid.clone(),
            action,
            telemetry_path,
            self.robot_name.clone(),
            self.robot_serial.clone(),
        )
        .await?;

        let mut telemetry_logger = self.telemetry_logger.lock().await;
        *telemetry_logger = Some(logger);
        drop(telemetry_logger);

        let mut pipeline_guard = self.pipeline.lock().await;
        *pipeline_guard = Some(pipeline);

        Ok(KClipStartResponse {
            clip_uuid: Some(new_uuid),
            error: None,
        })
    }

    async fn stop_kclip(&self) -> Result<KClipStopResponse> {
        // Get the UUID first
        let uuid = {
            let mut uuid_guard = self.kclip_uuid.lock().await;
            uuid_guard.take()
        };

        // Early return if no recording is active
        let uuid = match uuid {
            Some(uuid) => uuid,
            None => {
                return Ok(KClipStopResponse {
                    clip_uuid: None,
                    error: Some(Error {
                        code: ErrorCode::InvalidArgument as i32,
                        message: "No active KClip recording".to_string(),
                    }),
                })
            }
        };

        let action = {
            let mut action_guard = self.current_action.lock().await;
            action_guard.take().unwrap_or_else(|| "unknown".to_string())
        };

        // Get the paths
        let (telemetry_path, video_path, merged_path) = Self::recording_paths(&uuid, &action)?;

        // Stop telemetry logger
        if let Some(logger) = self.telemetry_logger.lock().await.take() {
            logger.stop().await?;
        }

        // Stop the pipeline
        let mut pipeline_guard = self.pipeline.lock().await;
        if let Some(pipeline) = pipeline_guard.as_ref() {
            // Get the bus
            let bus = pipeline
                .bus()
                .ok_or_else(|| eyre!("Failed to get pipeline bus"))?;

            // Send EOS event
            pipeline.send_event(gst::event::Eos::new());

            // Wait for EOS or Error message with a shorter timeout
            let timeout = gst::ClockTime::from_seconds(2);
            let msg =
                bus.timed_pop_filtered(timeout, &[gst::MessageType::Eos, gst::MessageType::Error]);

            match msg {
                Some(msg) => {
                    use gst::MessageView;
                    match msg.view() {
                        MessageView::Eos(..) => {
                            tracing::info!("Pipeline received EOS");
                        }
                        MessageView::Error(err) => {
                            tracing::error!(
                                "Pipeline error: {} ({})",
                                err.error(),
                                err.debug().unwrap_or_default()
                            );
                        }
                        _ => unreachable!(),
                    }
                }
                None => {
                    tracing::warn!("Timeout waiting for pipeline EOS");
                }
            }

            // Force state change to NULL immediately
            pipeline.set_state(gst::State::Null)?;

            // Clear the pipeline
            *pipeline_guard = None;

            let telemetry = Telemetry::try_get();
            if let Some(telemetry) = telemetry {
                telemetry.update_video_timestamp(0);
                telemetry.update_frame_number(0);
            }

            // Merge the video file
            combine_with_video(
                video_path.to_str().unwrap(),
                telemetry_path.to_str().unwrap(),
                merged_path.to_str().unwrap(),
            )?;

            // Clean up temporary files
            if let Err(e) = std::fs::remove_file(&video_path) {
                tracing::warn!("Failed to remove temporary video file: {}", e);
            }
            if let Err(e) = std::fs::remove_file(&telemetry_path) {
                tracing::warn!("Failed to remove temporary telemetry file: {}", e);
            }

            Ok(KClipStopResponse {
                clip_uuid: Some(uuid),
                error: None,
            })
        } else {
            Ok(KClipStopResponse {
                clip_uuid: None,
                error: Some(Error {
                    code: ErrorCode::Unknown as i32,
                    message: "Pipeline not found for active recording".to_string(),
                }),
            })
        }
    }
}
