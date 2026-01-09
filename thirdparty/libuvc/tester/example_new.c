#include "libuvc/libuvc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>

// 全局帧计数器
volatile int g_frame_count = 0;
// 全局运行标志，用于捕获中断信号
volatile int g_running = 1;

// 信号处理函数，捕获Ctrl+C等中断信号
void signal_handler(int signum) {
    printf("\n[调试] 捕获信号 %d，准备退出...\n", signum);
    g_running = 0;
}

static int print_usage(char *argv[])
{
    printf("Usage:\n");
    printf("%s [--help] [-h] [-u <usb device id>] [-s <widthxheight>] [-f <format>]\n", argv[0]);
    printf("\t[-i <index of control interface>] [-n <frames per second>] [-l]\n");
    printf("\t[-C, --get-ctrl <ctrl>[,<ctrl>...]] [-c, --set-ctrl <ctrl>=<val>[,<ctrl>=<val>...]]\n");
    printf("压测说明：程序会自动循环收150帧后重启\n");
    return 0;
}

static int print_uvc_ctrl_list(char *argv[]){
    printf("Support the followoing uvc control <ctrl:[support_val1,support_val2,...]>: \n (ep %s --get-ctrl ctrl --set-ctrl ctrl=support_val)\n", argv[0]);
    printf("    transfer_mode: [0: usb, 1: ethernet]\n");
    printf("    pc_fps: [10: 10Hz, 20: 20Hz, 30: 30Hz]\n");
    printf("    usb_mode_cfg: [0: uvc, 1: adb, 2: eth_bridge]\n");
    printf("    sys_time: [1: set x3 system time]\n");
    return 0;
}

// FPS计数器结构
typedef struct {
    unsigned int frame_count;
    double last_timestamp;
    double fps;
} FpsCounter;

void fps_counter_init(FpsCounter *counter) {
    if (counter != NULL) {
        counter->frame_count = 0;
        counter->last_timestamp = 0.0;
        counter->fps = 0.0;
    }
}

double get_current_system_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

double calculate_fps(FpsCounter *counter, double current_timestamp) {
    if (counter == NULL) return 0.0;

    if (counter->last_timestamp == 0.0) {
        counter->last_timestamp = current_timestamp;
        return 0.0;
    }

    double elapsed = current_timestamp - counter->last_timestamp;
    counter->last_timestamp = current_timestamp;

    if (elapsed > 0) {
        counter->fps = 1.0 / elapsed;
    }

    counter->frame_count++;

    return counter->fps;
}

// 帧回调函数
static void cb(uvc_frame_t *frame, void *ptr) {
    static FpsCounter fps_counter;
    static int initialized = 0;
    if (!initialized) {
        fps_counter_init(&fps_counter);
        initialized = 1;
        printf("[调试] 帧回调函数初始化完成\n");
    }

    double current_timestamp = get_current_system_time();
    double current_fps = calculate_fps(&fps_counter, current_timestamp);

    // 每10帧打印一次信息
    if (fps_counter.frame_count % 10 == 0) {
        printf("[调试] 帧率: %.2f FPS | 累计帧数: %d\n", current_fps, g_frame_count);
    }

    // 累加帧计数
    g_frame_count++;
    // 调试：每收到30帧打印一次进度
    if (g_frame_count % 30 == 0) {
        printf("[调试] 已接收 %d/150 帧\n", g_frame_count);
    }

    // BGR格式转换（保留但不显示）
    uvc_frame_t *bgr = uvc_allocate_frame(frame->width * frame->height * 3);
    if (!bgr) {
        printf("[错误] 无法分配BGR帧内存\n");
        return;
    }

    uvc_error_t ret;
    switch (frame->frame_format) {
        case UVC_FRAME_FORMAT_H264:
            break;
        case UVC_COLOR_FORMAT_MJPEG:
            break;
        case UVC_COLOR_FORMAT_YUYV:
            ret = uvc_any2bgr(frame, bgr);
            if (ret) {
                uvc_perror(ret, "[错误] uvc_any2bgr转换失败");
                uvc_free_frame(bgr);
                return;
            }
            break;
        default:
            break;
    }

    uvc_free_frame(bgr);
}

static enum uvc_frame_format convert_frame_format(const char *format) {
    char buffer[8];
    int max = sizeof(buffer) - 1;
    int i;

    for (i = 0; i < max; i++) {
        if (isalnum(format[i]))
            buffer[i] = format[i];
        else
            break;
    }
    buffer[i] = '\0';

    if (strcasecmp(buffer, "nv12") == 0) return UVC_FRAME_FORMAT_NV12;
    if (strcasecmp(buffer, "y16") == 0 || strcasecmp(buffer, "gray16") == 0) return UVC_FRAME_FORMAT_GRAY16;
    if (strcasecmp(buffer, "rgb24") == 0 || strcasecmp(buffer, "rgb") == 0) return UVC_FRAME_FORMAT_RGB;
    if (strcasecmp(buffer, "bgr24") == 0 || strcasecmp(buffer, "bgr") == 0) return UVC_FRAME_FORMAT_BGR;
    if (strcasecmp(buffer, "h264") == 0 || strcasecmp(buffer, "h.264") == 0) return UVC_FRAME_FORMAT_H264;
    if (strcasecmp(buffer, "yuyv") == 0 || strcasecmp(buffer, "yuy2") == 0) return UVC_FRAME_FORMAT_YUYV;
    if (strcasecmp(buffer, "xr24") == 0) return UVC_FRAME_FORMAT_XR24;

    return UVC_FRAME_FORMAT_UNKNOWN;
}

static int hanlder_uvc_ctrl_get_req(uvc_device_handle_t *devh, const char *req, void *value) {
    int ret = 0;
    if(strcasecmp(req, "pc_fps") == 0) {
        ret = uvc_get_point_cloud_fps(devh, value, UVC_GET_CUR);
        printf("[调试] 获取pc_fps控制值，返回码: %d\n", ret);
    } else if(strcasecmp(req, "transfer_mode") == 0) {
        ret = uvc_get_transfer_mode(devh, value, UVC_GET_CUR);
        printf("[调试] 获取transfer_mode控制值，返回码: %d\n", ret);
    } else if(strcasecmp(req, "usb_mode_cfg") == 0) {
        ret = uvc_get_usb_mode_cfg(devh, value, UVC_GET_CUR);
        printf("[调试] 获取usb_mode_cfg控制值，返回码: %d\n", ret);
    } else if(strcasecmp(req, "sys_time") == 0) {
        ret = uvc_get_sys_time(devh, value, UVC_GET_CUR);
        printf("[调试] 获取sys_time控制值，返回码: %d\n", ret);
    } else {
        printf("[警告] 未知的控制参数: %s\n", req);
        ret = -1;
    }
    return ret;
}

static int hanlder_uvc_ctrl_set_req(uvc_device_handle_t *devh, const char *req, void *value) {
    int ret = 0;
    if(strcasecmp(req, "pc_fps") == 0) {
        ret = uvc_set_point_cloud_fps(devh, *(uint8_t *)value);
        printf("[调试] 设置pc_fps控制值为%d，返回码: %d\n", *(uint8_t *)value, ret);
    } else if(strcasecmp(req, "transfer_mode") == 0) {
        ret = uvc_set_transfer_mode(devh, *(uint8_t *)value);
        printf("[调试] 设置transfer_mode控制值为%d，返回码: %d\n", *(uint8_t *)value, ret);
    } else if(strcasecmp(req, "usb_mode_cfg") == 0) {
        ret = uvc_set_usb_mode_cfg(devh, *(uint8_t *)value);
        printf("[调试] 设置usb_mode_cfg控制值为%d，返回码: %d\n", *(uint8_t *)value, ret);
    } else if(strcasecmp(req, "sys_time") == 0) {
        // ret = uvc_set_sys_time(devh, *(uint8_t *)value);
        printf("[调试] 设置sys_time控制值为%d，返回码: %d\n", *(uint8_t *)value, ret);
    } else {
        printf("[警告] 未知的控制参数: %s\n", req);
        ret = -1;
    }
    return ret;
}
#define POOL_SIZE 4
typedef struct {
  uint8_t *pool[POOL_SIZE];
  size_t size;
  int idx;
} FrameCtx;


static uint8_t* getData(size_t size, void *user_ptr)
{
    FrameCtx *ctx = (FrameCtx*)user_ptr;
    if (ctx->size == 0) {
        ctx->size = size;
        for (int i = 0; i < POOL_SIZE; i++) {
            ctx->pool[i] = (uint8_t*)malloc(size);
        }
    }
    uint8_t *buf = ctx->pool[ctx->idx];
    ctx->idx = (ctx->idx + 1) % POOL_SIZE;
    return buf;
}
void free_pool(FrameCtx *ctx)
{
    for (int i = 0; i < POOL_SIZE; i++) {
        free(ctx->pool[i]);
    }
}
static void putData(void *user_ptr)
{
    FrameCtx *ctx = (FrameCtx*)user_ptr;
    // 累加帧计数
    g_frame_count++;
    // 调试：每收到30帧打印一次进度
    if (g_frame_count % 30 == 0) {
        printf("[调试] 已接收 %d/150 帧\n", g_frame_count);
    }
    printf("g_frame_count: %d, [putData] Frame received, size=%zu\n",g_frame_count, ctx->size);
    // 不释放，等下次 getData 循环使用

      
}

int main(int argc, char **argv) {
  FrameCtx frame_ctx = {0};
    // 注册信号处理函数，捕获中断信号
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    printf("[调试] 主程序启动，注册信号处理函数完成\n");

    // UVC核心对象
    uvc_context_t *ctx = NULL;
    uvc_device_t *dev = NULL;
    uvc_device_handle_t *devh = NULL;
    uvc_stream_ctrl_t ctrl;
    uvc_error_t res;

    // 命令行参数默认值
    const char *ctl_format_str = NULL;
    const char *string;
    int usb_vid = 0x3840;
    int usb_pid = 0x1020;
    int usb_intf = -1;
    int ctl_width = 0;
    int ctl_height = 0;
    int ctl_fps = 0;
    enum uvc_frame_format ctl_format = UVC_FRAME_FORMAT_UNKNOWN;
    const char *get_ctrl = NULL;
    const char *set_ctrl = NULL;
    int is_uvc_ctrl_req = 0;
    int loop_count = 0;  // 循环计数器

    // 解析命令行参数
    printf("[调试] 开始解析命令行参数\n");
    for (int i = 1; i < argc; i++) {
        printf("[调试] 解析参数: %s\n", argv[i]);
        if (strcasecmp(argv[i], "--help") == 0 || strcasecmp(argv[i], "-h") == 0) {
            print_usage(argv);
            exit(0);
        } else if (strcmp(argv[i], "-u") == 0) {
            if ((++i) >= argc) break;
            if (argv[i][0] != ':') usb_vid = strtoul(argv[i], NULL, 16);
            string = strstr(argv[i], ":");
            if (string && string[1]) usb_pid = strtoul(string + 1, NULL, 16);
            printf("[调试] 设置USB设备ID: %04X:%04X\n", usb_vid, usb_pid);
        } else if (strcmp(argv[i], "-i") == 0) {
            if ((++i) >= argc) break;
            usb_intf = atoi(argv[i]);
            printf("[调试] 设置USB接口: %d\n", usb_intf);
        } else if (strcmp(argv[i], "-s") == 0) {
            if ((++i) >= argc) break;
            if (argv[i][0] != 'x') ctl_width = strtoul(argv[i], NULL, 10);
            string = strstr(argv[i], "x");
            if (string && string[1]) ctl_height = strtoul(string + 1, NULL, 10);
            printf("[调试] 设置分辨率: %dx%d\n", ctl_width, ctl_height);
        } else if (strcmp(argv[i], "-f") == 0) {
            if ((++i) >= argc) break;
            ctl_format_str = argv[i];
            ctl_format = convert_frame_format(ctl_format_str);
            if (ctl_format == UVC_FRAME_FORMAT_UNKNOWN) {
                printf("[警告] 忽略无效格式: %s\n", ctl_format_str);
                ctl_format_str = NULL;
            } else {
                printf("[调试] 设置格式: %s\n", ctl_format_str);
            }
        } else if (strcmp(argv[i], "-n") == 0) {
            if ((++i) >= argc) break;
            ctl_fps = atoi(argv[i]);
            printf("[调试] 设置帧率: %d\n", ctl_fps);
        } else if (strcmp(argv[i], "-C") == 0 || strcmp(argv[i], "--get-ctrl") == 0) {
            if ((++i) >= argc) break;
            get_ctrl = argv[i];
            is_uvc_ctrl_req = 1;
            printf("[调试] 设置获取控制参数: %s\n", get_ctrl);
        } else if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--set-ctrl") == 0) {
            if ((++i) >= argc) break;
            set_ctrl = argv[i];
            is_uvc_ctrl_req = 1;
            printf("[调试] 设置控制参数: %s\n", set_ctrl);
        } else if (strcmp(argv[i], "-l") == 0) {
            print_uvc_ctrl_list(argv);
            exit(0);
        } else {
            printf("[警告] 未知参数: %s\n", argv[i]);
        }
    }

    // 打印设备信息
    if (usb_intf < 0)
        printf("[调试] 目标设备: USB (%04X:%04X) [默认接口]\n", usb_vid, usb_pid);
    else
        printf("[调试] 目标设备: USB (%04X:%04X) [接口 %d]\n", usb_vid, usb_pid, usb_intf);

    // 初始化UVC上下文
    printf("[调试] 开始初始化UVC上下文\n");
    printf("[调试] UVC上下文初始化完成\n");

    // 压测无限循环
    while (g_running) {
    res = uvc_init(&ctx, NULL);
    if (res < 0) {
        uvc_perror(res, "[错误] uvc_init 失败");
        return res;
    }
        loop_count++;
        dev = NULL;
        devh = NULL;
        memset(&ctrl, 0, sizeof(uvc_stream_ctrl_t));

        // 记录循环开始时间
        time_t loop_start = time(NULL);
        struct tm *start_tm = localtime(&loop_start);
        char start_time[64];
        strftime(start_time, sizeof(start_time), "%Y-%m-%d %H:%M:%S", start_tm);
        printf("\n=============================================\n");
        printf("[调试] 压测循环 %d 开始 [%s]\n", loop_count, start_time);
        printf("=============================================\n");

        // 查找UVC设备
        printf("[调试] 开始查找UVC设备...\n");
        res = uvc_find_device(ctx, &dev, usb_vid, usb_pid, NULL);
        if (res < 0) {
            uvc_perror(res, "[错误] uvc_find_device 失败");
            printf("[调试] 等待1秒后重试...\n");
            sleep(1);
            continue;
        }
        printf("[调试] 找到UVC设备\n");

        // 打开UVC设备
        printf("[调试] 开始打开UVC设备...\n");
        res = uvc_open(dev, usb_intf, &devh);
        if (res < 0) {
            uvc_perror(res, "[错误] uvc_open 失败");
            uvc_unref_device(dev);
            printf("[调试] 等待1秒后重试...\n");
            sleep(1);
            continue;
        }
        printf("[调试] 打开UVC设备成功\n");
        uvc_print_diag(devh, stderr);

        // 处理控制请求
        // if (is_uvc_ctrl_req) {
        //     printf("[调试] 进入控制请求模式\n");
        //     if (get_ctrl != NULL) {
        //         printf("[调试] 获取控制参数: %s\n", get_ctrl);
        //         char *token = strtok((char *)get_ctrl, ",");
        //         while(token != NULL) {
        //             int value;
        //             res = hanlder_uvc_ctrl_get_req(devh, token, &value);
        //             if(res == UVC_SUCCESS) {
        //                 printf("[调试] %s = %d\n", token, value);
        //             } else {
        //                 uvc_perror(res, "[错误] 获取控制参数失败");
        //             }
        //             token = strtok(NULL, ",");
        //         }
        //     }

        //     if (set_ctrl != NULL) {
        //         printf("[调试] 设置控制参数: %s\n", set_ctrl);
        //         char *token = strtok((char *)set_ctrl, ",");
        //         while(token != NULL) {
        //             char *saveptr;
        //             char *key = strtok_r(token, "=", &saveptr);
        //             char *val = strtok_r(NULL, "=", &saveptr);
        //             if(key != NULL && val != NULL) {
        //                 int value = atoi(val);
        //                 res = hanlder_uvc_ctrl_set_req(devh, key, &value);
        //                 if(res!= UVC_SUCCESS) {
        //                     uvc_perror(res, "[错误] 设置控制参数失败");
        //                 } else {
        //                     printf("[调试] 成功设置 %s = %d\n", key, value);
        //                 }
        //             } else {
        //                 printf("[错误] 无效的控制参数格式: %s\n", token);
        //             }
        //             token = strtok(NULL, ",");
        //         }
        //     }

        //     // 控制请求处理完成后关闭设备并退出
        //     uvc_close(devh);
        //     printf("[调试] 控制请求处理完成，关闭设备\n");
        //     uvc_unref_device(dev);
        //     break;  // 退出循环，因为控制请求不是压测模式
        // }

        // 配置流参数
        printf("[调试] 开始配置流参数...\n");
        const uvc_format_desc_t *format_desc = uvc_get_format_descs(devh);
        if (!format_desc) {
            printf("[错误] 获取格式描述失败\n");
            uvc_close(devh);
            uvc_unref_device(dev);
            printf("[调试] 等待1秒后重试...\n");
            sleep(1);
            continue;
        }

        const uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
        enum uvc_frame_format frame_format;
        int width = 640;
        int height = 480;
        int fps = 30;

        // 确定帧格式
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
                    frame_format = (usb_intf == 0) ? UVC_FRAME_FORMAT_NV12 : 
                                  (usb_intf == 1) ? UVC_FRAME_FORMAT_GRAY16 : UVC_FRAME_FORMAT_YUYV;
                }
                break;
        }
        printf("[调试] 自动检测帧格式: %d\n", frame_format);

        // 确定分辨率和帧率
        if (frame_desc) {
            width = frame_desc->wWidth;
            height = frame_desc->wHeight;
            fps = 10000000 / frame_desc->dwDefaultFrameInterval;
            printf("[调试] 自动检测分辨率: %dx%d，帧率: %d\n", width, height, fps);

            // 匹配用户指定的帧率
            if (ctl_fps > 0 && frame_desc->intervals) {
                for (int i = 0; frame_desc->intervals[i] != 0; i++) {
                    if (10000000 / frame_desc->intervals[i] == ctl_fps) {
                        fps = ctl_fps;
                        printf("[调试] 匹配用户指定帧率: %d\n", fps);
                        break;
                    }
                }
            }
        }

        // 应用用户指定的参数
        if (ctl_width > 0) {
            printf("[调试] 应用用户指定宽度: %d\n", ctl_width);
            width = ctl_width;
        }
        if (ctl_height > 0) {
            printf("[调试] 应用用户指定高度: %d\n", ctl_height);
            height = ctl_height;
        }
        if (ctl_format != UVC_FRAME_FORMAT_UNKNOWN) {
            printf("[调试] 应用用户指定格式: %d\n", ctl_format);
            frame_format = ctl_format;
        }

        // 协商流控制参数
        printf("[调试] 开始协商流控制参数...\n");
        res = uvc_get_stream_ctrl_format_size(devh, &ctrl, frame_format, width, height, fps);
        if (res < 0) {
            uvc_perror(res, "[错误] 流参数协商失败");
            uvc_close(devh);
            uvc_unref_device(dev);
            printf("[调试] 等待1秒后重试...\n");
            sleep(1);
            continue;
        }
        printf("[调试] 流参数协商完成:\n");
        uvc_print_stream_ctrl(&ctrl, stderr);

        // 启动流传输并等待150帧
        g_frame_count = 0;
        printf("[调试] 启动流传输，等待接收150帧...\n");

        res = uvc_start_streaming(devh,  &ctrl, getData,putData, &frame_ctx , 0); 
        if (res < 0) {
            uvc_perror(res, "[错误] 启动流传输失败");
            uvc_close(devh);
            uvc_unref_device(dev);
            printf("[调试] 等待1秒后重试...\n");
            sleep(1);
            continue;
        }
        printf("[调试] 流传输启动成功\n");

        // 等待帧计数器达到150（带超时）
        time_t wait_start = time(NULL);
        printf("[调试] 开始等待帧接收，超时时间30秒\n");
        while (g_frame_count < 150 && g_running) {
            usleep(1000);  // 1ms休眠，降低CPU占用

            // 超时判断
            if (time(NULL) - wait_start > 30) {
                printf("[警告] 超时30秒！仅收到 %d 帧，强制终止流\n", g_frame_count);
                break;
            }
        }

        // 停止流传输
        printf("[调试] 停止流传输\n");
        uvc_stop_streaming(devh);
        printf("[调试] 流传输已停止，本次共收到 %d 帧\n", g_frame_count);

        // 关闭UVC设备
        printf("[调试] 关闭UVC设备\n");
        uvc_close(devh);

        // 释放设备描述符
        printf("[调试] 释放设备描述符\n");
        uvc_unref_device(dev);

        // 统计循环耗时
        time_t loop_end = time(NULL);
        double loop_duration = difftime(loop_end, loop_start);
        struct tm *end_tm = localtime(&loop_end);
        char end_time[64];
        strftime(end_time, sizeof(end_time), "%Y-%m-%d %H:%M:%S", end_tm);
        printf("=============================================\n");
        printf("[调试] 压测循环 %d 结束 [%s]\n", loop_count, end_time);
        printf("[调试] 本次循环耗时: %.1f 秒\n", loop_duration);
        printf("=============================================\n");

        // 如果收到退出信号，跳出循环
        if (!g_running) {
            printf("[调试] 收到退出信号，退出压测循环\n");
            break;
        }
        uvc_exit(ctx);
        system("usbreset 3840:1020");
        sleep(1);
    }

    // 清理资源
    printf("[调试] 开始清理资源\n");
    printf("[调试] UVC上下文已退出\n");
    printf("[调试] 程序正常结束\n");

    return 0;
}