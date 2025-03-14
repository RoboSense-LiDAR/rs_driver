#include "libuvc/libuvc.h"
#if defined(SDL2_FOUND) && (SDL2_FOUND != 0)
  #include <SDL2/SDL.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int print_usage(char *argv[])
{
  printf("Usage:\n");
  printf("%s [--help] [-h] [-u <usb device id>] [-s <widthxheight>] [-f <format>]\n", argv[0]);
  printf("\t[-i <index of control interface>] [-t <time to live in seconds>] [-l]\n");
  printf("\t[-C, --get-ctrl <ctrl>[,<ctrl>...]] [-c, --set-ctrl <ctrl>=<val>[,<ctrl>=<val>...]]\n");
  printf("Examples:\n");
  printf("%s -u \"1d6b:0104\" -i \"2\"\n", argv[0]);
  printf("%s -s \"960x144\" -f \"nv12\"\n", argv[0]);
  printf("%s --get-ctrl pc_fps\n", argv[0]);
  printf("%s --set-ctrl pc_fps=10,close_req=1\n", argv[0]);
  printf("%s -l to list the uvc ctrl list\n", argv[0]);
  return 0;
}

/* This is an AUTO-GENERATED file! Update it with the output of `ctrl-gen.py tool_help`. */
static int print_uvc_ctrl_list(char *argv[]){
    printf("Support the followoing uvc control <ctrl:[support_val1,support_val2,...]>: \n (ep %s --get-ctrl ctrl --set-ctrl ctrl=support_val)\n", argv[0]);
    printf("    transfer_mode: [0: usb, 1: ethernet]\n");
    printf("    pc_fps: [10: 10Hz, 20: 20Hz, 30: 30Hz]\n");
    printf("    usb_mode_cfg: [0: uvc, 1: adb, 2: eth_bridge]\n");
    printf("    sys_time: [1: set x3 system time]\n");
}

#if defined(SDL2_FOUND) && (SDL2_FOUND != 0)
static void display(uvc_frame_t *frame, SDL_Renderer *renderer) {
  SDL_Texture *texture = NULL;

  if (renderer == NULL)
    return;

  switch (frame->frame_format) {
    case UVC_COLOR_FORMAT_NV12:
      texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_NV12, SDL_TEXTUREACCESS_STREAMING, frame->width, frame->height);
    break;
  }

  if (texture == NULL) {
    uvc_perror(UVC_ERROR_IO, "create sdl texture");
    return;
  }

  SDL_UpdateTexture(texture, NULL, frame->data, frame->step);

  SDL_RenderClear(renderer);
  SDL_RenderCopy(renderer, texture, NULL, NULL);
  SDL_RenderPresent(renderer);

  SDL_DestroyTexture(texture);
}
#endif

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
static void cb(uvc_frame_t *frame, void *ptr) {
  uvc_frame_t *bgr;
  uvc_error_t ret;
  enum uvc_frame_format *frame_format = (enum uvc_frame_format *)ptr;
  /* FILE *fp;
   * static int jpeg_count = 0;
   * static const char *H264_FILE = "iOSDevLog.h264";
   * static const char *MJPEG_FILE = ".jpeg";
   * char filename[16]; */

  /* We'll convert the image from YUV/JPEG to BGR, so allocate space */
  bgr = uvc_allocate_frame(frame->width * frame->height * 3);
  if (!bgr) {
    printf("unable to allocate bgr frame!\n");
    return;
  }
  printf("Sec: [%ld | %lx]\r\n",frame->capture_time.tv_sec,frame->capture_time.tv_sec);
  printf("MicroSec: [%ld | %lx]\r\n",frame->capture_time.tv_usec,frame->capture_time.tv_usec);
  printf("callback! frame_format = %d, width = %d, height = %d, length = %lu, ptr = %p\n",
    frame->frame_format, frame->width, frame->height, frame->data_bytes, ptr);

  switch (frame->frame_format) {
  case UVC_FRAME_FORMAT_H264:
    /* use `ffplay H264_FILE` to play */
    /* fp = fopen(H264_FILE, "a");
     * fwrite(frame->data, 1, frame->data_bytes, fp);
     * fclose(fp); */
    break;
  case UVC_COLOR_FORMAT_MJPEG:
    /* sprintf(filename, "%d%s", jpeg_count++, MJPEG_FILE);
     * fp = fopen(filename, "w");
     * fwrite(frame->data, 1, frame->data_bytes, fp);
     * fclose(fp); */
    break;
  case UVC_COLOR_FORMAT_YUYV:
    /* Do the BGR conversion */
    ret = uvc_any2bgr(frame, bgr);
    if (ret) {
      uvc_perror(ret, "uvc_any2bgr");
      uvc_free_frame(bgr);
      return;
    }
    break;
  default:
    break;
  }

  if (frame->sequence % 30 == 0) {
    printf(" * got image %u\n",  frame->sequence);
  }

  /* Call a user function:
   *
   * my_type *my_obj = (*my_type) ptr;
   * my_user_function(ptr, bgr);
   * my_other_function(ptr, bgr->data, bgr->width, bgr->height);
   */

  /* Call a C++ method:
   *
   * my_type *my_obj = (*my_type) ptr;
   * my_obj->my_func(bgr);
   */

  /* Use opencv.highgui to display the image:
   * 
   * cvImg = cvCreateImageHeader(
   *     cvSize(bgr->width, bgr->height),
   *     IPL_DEPTH_8U,
   *     3);
   *
   * cvSetData(cvImg, bgr->data, bgr->width * 3); 
   *
   * cvNamedWindow("Test", CV_WINDOW_AUTOSIZE);
   * cvShowImage("Test", cvImg);
   * cvWaitKey(10);
   *
   * cvReleaseImageHeader(&cvImg);
   */

#if defined(SDL2_FOUND) && (SDL2_FOUND != 0)
  display(frame, ptr);
#endif

  uvc_free_frame(bgr);
}

static enum uvc_frame_format convert_frame_format(const char *format) {
  char buffer[5];

  memcpy(buffer, format, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';

  if (strcasecmp(buffer, "nv12") == 0)
    return UVC_FRAME_FORMAT_NV12;
  if (strcasecmp(buffer, "y16") == 0)
    return UVC_FRAME_FORMAT_GRAY16;
  if (strcasecmp(buffer, "gray16") == 0)
    return UVC_FRAME_FORMAT_GRAY16;
  if (strcasecmp(buffer, "rgb24") == 0)
    return UVC_FRAME_FORMAT_RGB;
  if (strcasecmp(buffer, "rgb") == 0)
    return UVC_FRAME_FORMAT_RGB;
  if (strcasecmp(buffer, "h264") == 0)
    return UVC_FRAME_FORMAT_H264;
  if (strcasecmp(buffer, "h.264") == 0)
    return UVC_FRAME_FORMAT_H264;
  if (strcasecmp(buffer, "yuyv") == 0)
    return UVC_FRAME_FORMAT_YUYV;
  if (strcasecmp(buffer, "yuy2") == 0)
    return UVC_FRAME_FORMAT_YUYV;

  return UVC_FRAME_FORMAT_UNKNOWN;
}

static int hanlder_uvc_ctrl_get_req(uvc_device_handle_t *devh, const char *req, void *value) {
  int ret = 0;
  if(strcasecmp(req, "pc_fps") == 0) {
    ret = uvc_get_point_cloud_fps(devh, value, UVC_GET_CUR);
  } else if(strcasecmp(req, "transfer_mode") == 0) {
    ret = uvc_get_transfer_mode(devh, value, UVC_GET_CUR);
  } else if(strcasecmp(req, "usb_mode_cfg") == 0) {
    ret = uvc_get_usb_mode_cfg(devh, value, UVC_GET_CUR);
  } else if(strcasecmp(req, "sys_time") == 0) {
    ret = uvc_get_sys_time(devh, value, UVC_GET_CUR);
  }
  return 0;
}

static int hanlder_uvc_ctrl_set_req(uvc_device_handle_t *devh, const char *req, void *value) {
  int ret = 0;
  if(strcasecmp(req, "pc_fps") == 0) {
    ret = uvc_set_point_cloud_fps(devh, *(uint8_t *)value);
  } else if(strcasecmp(req, "transfer_mode") == 0) {
    ret = uvc_set_transfer_mode(devh, *(uint8_t *)value);
  } else if(strcasecmp(req, "usb_mode_cfg") == 0) {
    ret = uvc_set_usb_mode_cfg(devh, *(uint8_t *)value);
  }
  return 0;
}

int main(int argc, char **argv) {
#if defined(SDL2_FOUND) && (SDL2_FOUND != 0)
  SDL_Window *window = NULL;
  SDL_Renderer *renderer = NULL;
  int sdl_initialized = 0;
#endif
  uvc_context_t *ctx;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;
  uvc_error_t res;
  const char *ctl_format_str = NULL;
  const char *string;
  int usb_vid = 0x1d6b;
  int usb_pid = 0x0104;
  int usb_intf = -1;
  int ctl_width = 0;
  int ctl_height = 0;
  int live_time = 10;
  enum uvc_frame_format ctl_format;
  int i;
  const char *get_ctrl = NULL;
  const char *set_ctrl = NULL;
  int is_uvc_ctrl_req = 0;

  for (i = 1; i < argc; i++) {
    if (strcasecmp(argv[i], "--help") == 0) {
      print_usage(argv);
      exit(0);
    } else if (strcasecmp(argv[i], "-h") == 0) {
      print_usage(argv);
      exit(0);
    } else if (strcmp(argv[i], "-u") == 0) {
      if ((++i) >= argc)
        break;
      if (argv[i][0] != ':')
        usb_vid = strtoul(argv[i], NULL, 16);
      string = strstr(argv[i], ":");
      if ((string != NULL) && (string[1] != '\0'))
        usb_pid = strtoul(string + 1, NULL, 16);
    } else if (strcmp(argv[i], "-i") == 0) {
      if ((++i) >= argc)
        break;
      usb_intf = atoi(argv[i]);
    } else if (strcmp(argv[i], "-s") == 0) {
      if ((++i) >= argc)
        break;
      if (argv[i][0] != 'x')
        ctl_width = strtoul(argv[i], NULL, 10);
      string = strstr(argv[i], "x");
      if ((string != NULL) && (string[1] != '\0'))
        ctl_height = strtoul(string + 1, NULL, 10);
    } else if (strcmp(argv[i], "-f") == 0) {
      if ((++i) >= argc)
        break;
      ctl_format_str = argv[i];
      ctl_format = convert_frame_format(ctl_format_str);
      if (ctl_format == UVC_FRAME_FORMAT_UNKNOWN) {
        printf("ignore invalid format: %s\n", ctl_format_str);
        ctl_format_str = NULL;
      }
    } else if (strcmp(argv[i], "-t") == 0) {
      if ((++i) >= argc)
        break;
      live_time = atoi(argv[i]);
    } else if (strcmp(argv[i], "-C") == 0) {
      if ((++i) >= argc)
        break;
      get_ctrl = argv[i];
      is_uvc_ctrl_req = 1;
    } else if (strcmp(argv[i], "--get-ctrl") == 0) {
      if ((++i) >= argc)
        break;
      get_ctrl = argv[i];
      is_uvc_ctrl_req = 1;
    } else if (strcmp(argv[i], "-c") == 0) {
      if ((++i) >= argc)
        break;
      set_ctrl = argv[i];
      is_uvc_ctrl_req = 1;
    } else if (strcmp(argv[i], "--set-ctrl") == 0) {
      if ((++i) >= argc)
        break;
      set_ctrl = argv[i];
      is_uvc_ctrl_req = 1;
    } else if (strcmp(argv[i], "-l") == 0) {
      print_uvc_ctrl_list(argv);
      exit(0);
    }
  }

  if (ctl_format_str == NULL)
    ctl_format = UVC_FRAME_FORMAT_UNKNOWN;

  if (usb_intf < 0)
    printf("usb (%04X:%04X) @ default\n", usb_vid, usb_pid);
  else
    printf("usb (%04X:%04X) @ %d\n", usb_vid, usb_pid, usb_intf);

  /* Initialize a UVC service context. Libuvc will set up its own libusb
   * context. Replace NULL with a libusb_context pointer to run libuvc
   * from an existing libusb context. */
  res = uvc_init(&ctx, NULL);

  if (res < 0) {
    uvc_perror(res, "uvc_init");
    return res;
  }

  puts("UVC initialized");

  /* Locates the first attached UVC device, stores in dev */
  res = uvc_find_device(
      ctx, &dev,
      usb_vid, /* filter devices: vendor_id, product_id, "serial_num" */
      usb_pid, NULL);

  if (res < 0) {
    uvc_perror(res, "uvc_find_device"); /* no devices found */
  } else {
    puts("Device found");

    /* Try to open the device: requires exclusive access */
    res = uvc_open(dev, usb_intf, &devh);

    if (res < 0) {
      uvc_perror(res, "uvc_open"); /* unable to open device */
    } else if(is_uvc_ctrl_req) {
        if (get_ctrl != NULL) {
          printf("Getting control(s): %s\n", get_ctrl);
          char *token = strtok((char *)get_ctrl, ",");
          while(token != NULL) {
            int value;
            res = hanlder_uvc_ctrl_get_req(devh, token, &value);
            if(res == UVC_SUCCESS) {
              printf("%s = %d\n", token, value);
            } else {
              uvc_perror(res, ".. hanlder_uvc_ctrl_get_req getting data");
            }
            token = strtok(NULL, ",");
          }
        }

        if (set_ctrl != NULL) {
          printf("Setting control(s): %s\n", set_ctrl);
          char *token = strtok((char *)set_ctrl, ",");
          while(token != NULL) {
            char *saveptr;
            char *key = strtok_r(token, "=", &saveptr);
            char *val = strtok_r(NULL, "=", &saveptr);
            if(key != NULL && val != NULL) {
              int value = atoi(val);
              res = hanlder_uvc_ctrl_set_req(devh, key, &value);
              if(res!= UVC_SUCCESS) {
                uvc_perror(res, ".. hanlder_uvc_ctrl_set_req setting data");
              }
            }
            token = strtok(NULL, ",");
          }

          /* Release our handle on the device */
          uvc_close(devh);
          puts("Device closed");
        }
    } else {
      puts("Device opened");

      /* Print out a message containing all the information that libuvc
       * knows about the device */
      uvc_print_diag(devh, stderr);

      const uvc_format_desc_t *format_desc = uvc_get_format_descs(devh);
      const uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
      enum uvc_frame_format frame_format;
      int width = 640;
      int height = 480;
      int fps = 30;

      switch (format_desc->bDescriptorSubtype) {
      case UVC_VS_FORMAT_MJPEG:
        frame_format = UVC_COLOR_FORMAT_MJPEG;
        break;
      case UVC_VS_FORMAT_FRAME_BASED:
        frame_format = UVC_FRAME_FORMAT_H264;
        break;
      default:
        frame_format = convert_frame_format(format_desc->fourccFormat);
        if (frame_format == UVC_FRAME_FORMAT_UNKNOWN) {
          if (usb_intf == 0)
            frame_format = UVC_FRAME_FORMAT_NV12;
          else if (usb_intf == 1)
            frame_format = UVC_FRAME_FORMAT_GRAY16;
          else
            frame_format = UVC_FRAME_FORMAT_YUYV;
        }
        break;
      }

      if (frame_desc) {
        width = frame_desc->wWidth;
        height = frame_desc->wHeight;
        fps = 10000000 / frame_desc->dwDefaultFrameInterval;
      }

      printf("\nFirst format: (%4s) %dx%d %dfps\n", format_desc->fourccFormat, width, height, fps);

      if (ctl_width > 0) {
        printf("ctl width: %d\n", ctl_width);
        width = ctl_width;
      }

      if (ctl_height > 0) {
        printf("ctl height: %d\n", ctl_height);
        height = ctl_height;
      }

      if (ctl_format != UVC_FRAME_FORMAT_UNKNOWN) {
        printf("ctl format: %s\n", ctl_format_str);
        frame_format = ctl_format;
      }

      /* Try to negotiate first stream profile */
      res = uvc_get_stream_ctrl_format_size(
          devh, &ctrl, /* result stored in ctrl */
          frame_format,
          width, height, fps /* width, height, fps */
      );

      /* Print out the result */
      uvc_print_stream_ctrl(&ctrl, stderr);

      if (res < 0) {
        uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
      } else {
        do {
#if defined(SDL2_FOUND) && (SDL2_FOUND != 0)
          if (usb_intf <= 0) { //Default or camera
            if (SDL_Init(SDL_INIT_VIDEO) < 0) {
              uvc_perror(UVC_ERROR_IO, "init sdl");
              break;
            }
            sdl_initialized = 1;

            window = SDL_CreateWindow("Streaming", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1920, 1080, SDL_WINDOW_SHOWN);
            if (window == NULL) {
              uvc_perror(UVC_ERROR_IO, "create sdl window");
              break;
            }

            renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
            if (renderer == NULL) {
              uvc_perror(UVC_ERROR_IO, "create sdl renderer");
              break;
            }
          } else {
            renderer = NULL;
          }
#endif
        } while (0);

        /* Start the video stream. The library will call user function cb:
         *   cb(frame, (void *) argument)
         */
#if defined(SDL2_FOUND) && (SDL2_FOUND != 0)
        res = uvc_start_streaming(devh, &ctrl, cb, renderer, 0);
#else
        res = uvc_start_streaming(devh, &ctrl, cb, NULL, 0);
#endif

        if (res < 0) {
          uvc_perror(res, "start_streaming"); /* unable to start stream */
        } else {
          puts("Streaming...");

          /* enable auto exposure - see uvc_set_ae_mode documentation */
          puts("Enabling auto exposure ...");
          const uint8_t UVC_AUTO_EXPOSURE_MODE_AUTO = 2;
          res = uvc_set_ae_mode(devh, UVC_AUTO_EXPOSURE_MODE_AUTO);
          if (res == UVC_SUCCESS) {
            puts(" ... enabled auto exposure");
          } else if (res == UVC_ERROR_PIPE) {
            /* this error indicates that the camera does not support the full AE mode;
             * try again, using aperture priority mode (fixed aperture, variable exposure time) */
            puts(" ... full AE not supported, trying aperture priority mode");
            const uint8_t UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY = 8;
            res = uvc_set_ae_mode(devh, UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY);
            if (res < 0) {
              uvc_perror(res, " ... uvc_set_ae_mode failed to enable aperture priority mode");
            } else {
              puts(" ... enabled aperture priority auto exposure mode");
            }
          } else {
            uvc_perror(res, " ... uvc_set_ae_mode failed to enable auto exposure mode");
          }

          sleep(live_time);

          /* End the stream. Blocks until last callback is serviced */
          uvc_stop_streaming(devh);
          puts("Done streaming.");
        }
      }

      /* Release our handle on the device */
      uvc_close(devh);
      puts("Device closed");
    }

    /* Release the device descriptor */
    uvc_unref_device(dev);
  }

  /* Close the UVC context. This closes and cleans up any existing device handles,
   * and it closes the libusb context if one was not provided. */
  uvc_exit(ctx);
  puts("UVC exited");

#if defined(SDL2_FOUND) && (SDL2_FOUND != 0)
  if (renderer != NULL)
    SDL_DestroyRenderer(renderer);
  if (window != NULL)
    SDL_DestroyWindow(window);
  if (sdl_initialized != 0)
    SDL_Quit();
#endif

  return 0;
}

