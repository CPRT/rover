#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <gst/gst.h>


typedef struct
{
  GMainLoop *loop;
  GstElement *pipeline;
  guint bus_watch;
  guint io_watch_id;
} GlobalData;

static gboolean handle_bus_msg (GstBus * bus, GstMessage * msg, GlobalData * data);
static gboolean io_callback (GIOChannel * io, GIOCondition condition, GlobalData * data);

int
main (int argc, char *argv[])
{
  GlobalData data;
  GIOChannel *io = NULL;
  GstBus *bus;
  GError *error = NULL;

  /* Initialize GStreamer */
  gst_init (&argc, &argv);

  if (argc < 2) {
    g_print ("Usage: %s <SRT_URI>\n", argv[0]);
    g_print ("Example: %s \"srt://:7001?mode=listener\"\n", argv[0]);
    g_print ("Controls:\n"
        "  'q': Quit application\n");
  }


// srtsrc -> av1parse -> decodebin -> videoconvert -> sink
  const char *default_uri = "srt://:7001?mode=listener";
  const char *uri = (argc >= 2) ? argv[1] : default_uri;

  g_print ("Trying to listen on URI: %s\n", uri);

  gchar *pipeline_str = g_strdup_printf (
      "srtsrc uri=\"%s\" latency=200 ! "
      "typefind ! "
      "av1parse ! "
      "decodebin ! "
      "videoconvert ! "
      "autovideosink sync=false",
      uri);

  data.pipeline = gst_parse_launch (pipeline_str, &error);
  g_free (pipeline_str);

  if (error) {
    g_printerr ("Failed to parse pipeline: %s\n", error->message);
    g_error_free (error);
    return -1;
  }

  /* Connect to the bus to receive callbacks */
  bus = gst_element_get_bus (data.pipeline);
  data.bus_watch = gst_bus_add_watch (bus, (GstBusFunc) handle_bus_msg, &data);
  gst_object_unref (bus);

  /* Set up the main loop */
  data.loop = g_main_loop_new (NULL, FALSE);

  /* Start playing */
  gst_element_set_state (data.pipeline, GST_STATE_PLAYING);
  g_print ("Pipeline started. Waiting for stream...\n");

  /* Listen to stdin input (Keyboard) */
  io = g_io_channel_unix_new (fileno (stdin));
  data.io_watch_id = g_io_add_watch (io, G_IO_IN, (GIOFunc) (io_callback), &data);
  g_io_channel_unref (io);

  /* Run the mainloop */
  g_main_loop_run (data.loop);

  /* Clean up */
  g_print ("Cleaning up...\n");
  g_source_remove (data.bus_watch);
  g_source_remove (data.io_watch_id);
  gst_element_set_state (data.pipeline, GST_STATE_NULL);
  gst_object_unref (data.pipeline);
  g_main_loop_unref (data.loop);

  return 0;
}

static gboolean
handle_bus_msg (GstBus * bus, GstMessage * msg, GlobalData * data)
{
  switch (GST_MESSAGE_TYPE (msg)) {
    case GST_MESSAGE_EOS:{
      g_print ("End of stream.\n");
      g_main_loop_quit (data->loop);
      break;
    }
    case GST_MESSAGE_ERROR:{
      GError *err = NULL;
      gchar *dbg_info = NULL;

      gst_message_parse_error (msg, &err, &dbg_info);
      g_printerr ("ERROR from element %s: %s\n",
          GST_OBJECT_NAME (msg->src), err->message);
      g_printerr ("Debugging info: %s\n", (dbg_info) ? dbg_info : "none");
      
      g_error_free (err);
      g_free (dbg_info);

      g_main_loop_quit (data->loop);
      break;
    }
    case GST_MESSAGE_STATE_CHANGED: {
        GstState old_state, new_state, pending_state;
        gst_message_parse_state_changed (msg, &old_state, &new_state, &pending_state);
        if (GST_MESSAGE_SRC (msg) == GST_OBJECT (data->pipeline)) {
            g_print ("Pipeline state changed from %s to %s\n",
                gst_element_state_get_name (old_state),
                gst_element_state_get_name (new_state));
        }
        break;
    }

    default:
      break;
  }

  return TRUE;
}

static gboolean
io_callback (GIOChannel * io, GIOCondition condition, GlobalData * data)
{
  gchar in;
  GError *error = NULL;

  switch (g_io_channel_read_chars (io, &in, 1, NULL, &error)) {
    case G_IO_STATUS_NORMAL:
      switch (in) {
        case 'q':
          g_main_loop_quit (data->loop);
          break;
      }
      break;
    case G_IO_STATUS_AGAIN:
      break;
    case G_IO_STATUS_ERROR:
      g_printerr ("stdin IO error: %s\n", error->message);
      g_error_free (error);
      g_main_loop_quit (data->loop);
      return FALSE;
    default:
      return FALSE;
  }

  return TRUE;
}