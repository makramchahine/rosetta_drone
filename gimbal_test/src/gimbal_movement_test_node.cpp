//INCLUDE
#pragma once

#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/GimbalAction.h>
#include <dji_osdk_ros/CameraTapZoomPoint.h>
#include <dji_osdk_ros/CameraSetZoomPara.h>
#include <dji_osdk_ros/CameraZoomCtrl.h>

#include <iostream>
#include <dji_camera_image.hpp>
#include <dji_osdk_ros/SetupCameraStream.h>
#include <dji_osdk_ros/SetupCameraH264.h>
#include <sensor_msgs/Image.h>

#include <geometry_msgs/Vector3.h>
#include <vector>

extern "C"{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#ifdef OPEN_CV_INSTALLED
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#endif

using namespace dji_osdk_ros;

AVCodecContext*       pCodecCtx;
AVCodec*              pCodec;
AVCodecParserContext* pCodecParserCtx;
SwsContext*           pSwsCtx;
AVFrame* pFrameYUV;
AVFrame* pFrameRGB;
uint8_t* rgbBuf;
size_t   bufSize;

bool ffmpeg_init()
{
    avcodec_register_all();
    pCodecCtx = avcodec_alloc_context3(NULL);
    if (!pCodecCtx)
    {
        return false;
    }

    pCodecCtx->thread_count = 4;
    pCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!pCodec || avcodec_open2(pCodecCtx, pCodec, NULL) < 0)
    {
        return false;
    }

    pCodecParserCtx = av_parser_init(AV_CODEC_ID_H264);
    if (!pCodecParserCtx)
    {
        return false;
    }

    pFrameYUV = av_frame_alloc();
    if (!pFrameYUV)
    {
        return false;
    }

    pFrameRGB = av_frame_alloc();
    if (!pFrameRGB)
    {
        return false;
    }

    pSwsCtx = NULL;

    pCodecCtx->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;
}

#ifdef SDL2_INSTALLED
#include <SDL2/SDL.h>
void sdl_show_rgb(uint8_t *rgb24Buf, int width, int height) {
  static SDL_Renderer* sdlRenderer = NULL;
  static SDL_Texture* sdlTexture = NULL;
  static SDL_Window *screen = NULL;
  static int initFlag = 0;

  if (initFlag == 0) {
    initFlag = 1;
    if(SDL_Init(SDL_INIT_VIDEO)) {
      printf( "Could not initialize SDL - %s\n", SDL_GetError());
      return;
    }


    //SDL 2.0 Support for multiple windows
    screen = SDL_CreateWindow("Gimbal_Movement_Test_Node", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                              100, 100,SDL_WINDOW_OPENGL|SDL_WINDOW_SHOWN|SDL_WINDOW_RESIZABLE);
    if(!screen) {
      printf("SDL: could not create window - exiting:%s\n",SDL_GetError());
      return;
    }
    sdlRenderer = SDL_CreateRenderer(screen, -1, 0);
    uint32_t pixformat = SDL_PIXELFORMAT_RGB24;
    sdlTexture = SDL_CreateTexture(sdlRenderer,pixformat, SDL_TEXTUREACCESS_STREAMING, width, height);
  }

  if (!sdlRenderer || !sdlTexture || !screen) return;
  SDL_SetWindowSize(screen, width, height);

  SDL_Event event;
  event.type = (SDL_USEREVENT + 1);
  SDL_PushEvent(&event);
  if (SDL_WaitEventTimeout(&event, 5)) {
    SDL_Rect sdlRect;
    SDL_UpdateTexture(sdlTexture, NULL, rgb24Buf, width * 3);
    sdlRect.x = 0;
    sdlRect.y = 0;
    sdlRect.w = width;
    sdlRect.h = height;

    SDL_RenderClear(sdlRenderer);
    SDL_RenderCopy(sdlRenderer, sdlTexture, NULL, &sdlRect);
    SDL_RenderPresent(sdlRenderer);
  }
}
#endif

void show_rgb(CameraRGBImage img, char* name)
{
  //std::cout << "#### Got image from:\t" << std::string(name) << std::endl;
#ifdef SDL2_INSTALLED
  sdl_show_rgb(img.rawData.data(), img.width, img.height);
#elif defined(OPEN_CV_INSTALLED)
  cv::Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width*3);
  cvtColor(mat, mat, cv::COLOR_RGB2BGR);
  imshow(name,mat);
  cv::waitKey(1);
#endif
}

void decodeToDisplay(uint8_t *buf, int bufLen)
{
    uint8_t* pData   = buf;
    int remainingLen = bufLen;
    int processedLen = 0;

    AVPacket pkt;
    av_init_packet(&pkt);
    while (remainingLen > 0)
    {
        processedLen = av_parser_parse2(pCodecParserCtx, pCodecCtx,
                                        &pkt.data, &pkt.size,
                                        pData, remainingLen,
                                        AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
        remainingLen -= processedLen;
        pData        += processedLen;

        if (pkt.size > 0)
        {
            int gotPicture = 0;
            avcodec_decode_video2(pCodecCtx, pFrameYUV, &gotPicture, &pkt);

            if (!gotPicture)
            {
                //DSTATUS_PRIVATE("Got Frame, but no picture\n");
                continue;
            }
            else
            {
                int w = pFrameYUV->width;
                int h = pFrameYUV->height;
                //DSTATUS_PRIVATE("Got picture! size=%dx%d\n", w, h);

                if(NULL == pSwsCtx)
                {
                    pSwsCtx = sws_getContext(w, h, pCodecCtx->pix_fmt,
                                             w, h, AV_PIX_FMT_RGB24,
                                             4, NULL, NULL, NULL);
                }

                if(NULL == rgbBuf)
                {
                    bufSize = avpicture_get_size(AV_PIX_FMT_RGB24, w, h);
                    rgbBuf = (uint8_t*) av_malloc(bufSize);
                    avpicture_fill((AVPicture*)pFrameRGB, rgbBuf, AV_PIX_FMT_RGB24, w, h);
                }

                if(NULL != pSwsCtx && NULL != rgbBuf)
                {
                    sws_scale(pSwsCtx,
                              (uint8_t const *const *) pFrameYUV->data, pFrameYUV->linesize, 0, pFrameYUV->height,
                              pFrameRGB->data, pFrameRGB->linesize);

                    pFrameRGB->height = h;
                    pFrameRGB->width = w;
#ifdef SDL2_INSTALLED
                    sdl_show_rgb(pFrameRGB->data[0], pFrameRGB->width, pFrameRGB->height);
#elif defined(OPEN_CV_INSTALLED)
                    cv::Mat mat(pFrameRGB->height, pFrameRGB->width, CV_8UC3, pFrameRGB->data[0], pFrameRGB->width * 3);
                    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
                    cv::imshow("camera_stream_node", mat);
                    cv::waitKey(1);
#endif
                }
            }
        }
    }
    av_free_packet(&pkt);
}

void mainCameraStreamCallBack(const sensor_msgs::Image& msg)
{
  CameraRGBImage img;
  img.rawData = msg.data;
  img.height  = msg.height;
  img.width   = msg.width;
  char Name[] = "MAIN_CAM";
  show_rgb(img, Name);
  //std::cout<<"height is"<<msg.height<<std::endl;
  //std::cout<<"width is"<<msg.width<<std::endl;
}


//CODE

int main(int argc, char** argv) {
  ros::init(argc, argv, "gimbal_movement_test_node");
  ros::NodeHandle nh;
  ros::Publisher gimbal_angle_publisher_ = nh.advertise<geometry_msgs::Vector3>("gimbal_test_angle", 10);

  auto gimbal_control_client = nh.serviceClient<GimbalAction>("gimbal_task_control");
  auto camera_set_tap_zoom_point_client = nh.serviceClient<CameraTapZoomPoint>("camera_task_tap_zoom_point");
  auto camera_set_zoom_para_client = nh.serviceClient<CameraSetZoomPara>("camera_task_set_zoom_para");
  auto camera_task_zoom_ctrl_client = nh.serviceClient<CameraZoomCtrl>("camera_task_zoom_ctrl");

  auto setup_camera_h264_client = nh.serviceClient<dji_osdk_ros::SetupCameraH264>("setup_camera_h264");
  //auto fpv_camera_h264_sub = nh.subscribe("dji_osdk_ros/camera_h264_stream", 10, cameraH264CallBack);
  dji_osdk_ros::SetupCameraH264 setupCameraH264_;
  ffmpeg_init();

  /*! RGB flow init */
  auto setup_camera_stream_client = nh.serviceClient<dji_osdk_ros::SetupCameraStream>("setup_camera_stream");
  auto main_camera_stream_sub = nh.subscribe("dji_osdk_ros/main_camera_images", 10, mainCameraStreamCallBack);
  dji_osdk_ros::SetupCameraStream setupCameraStream_;

  float groll=0, gpitch=0, gyaw=0;
  GimbalAction gimbalAction;
  gimbalAction.request.is_reset = false;
  gimbalAction.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
  gimbalAction.request.rotationMode = 0;
  gimbalAction.request.pitch = gpitch;
  gimbalAction.request.roll = groll;
  gimbalAction.request.yaw = gyaw;
  gimbalAction.request.time = 0.5;
  gimbal_control_client.call(gimbalAction);

  geometry_msgs::Vector3 gimbalv;
  gimbalv.x=gpitch;
  gimbalv.y=groll;
  gimbalv.z=gyaw;
  gimbal_angle_publisher_.publish(gimbalv);
  ros::spinOnce();

  setupCameraStream_.request.cameraType = setupCameraStream_.request.MAIN_CAM;
  setupCameraStream_.request.start = 1;
  setup_camera_stream_client.call(setupCameraStream_);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  /*! sample loop start */
  char inputChar = 0;
  char inChar = 0;
  //int a=0;
  while (true) {
    std::cout << std::endl;
    std::cout
        << "| Available commands:                                            |\n"
        << "| [f] Set camera tap zoom point                                  |\n"
        << "|     Vice camera : Z30, H20/H20T(zoom mode) etc.                |\n"
        << "| [g] Set camera zoom parameter                                  |\n"
        << "|     Vice camera : Z30, H20/H20T(zoom mode) etc.                |\n"
        << "| [m] Rotate gimbal sample                                       |\n"
        << "|     Main camera : X5S, X7, XTS, Z30, H20/H20T etc.             |\n"
        << "| [n] Reset gimbal sample                                        |\n"
        << "|     Main camera : X5S, X7, XTS, Z30, H20/H20T etc.             |\n"
        << "| [q] Quit                                                       |\n";
    std::cin >> inputChar;
    switch (inputChar)
    {
      case 'f': {
        CameraTapZoomPoint cameraTapZoomPoint;
        cameraTapZoomPoint.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_1);
        cameraTapZoomPoint.request.multiplier = 5;
        cameraTapZoomPoint.request.x = 0.3;
        cameraTapZoomPoint.request.y = 0.3;
        camera_set_tap_zoom_point_client.call(cameraTapZoomPoint);
        sleep(5);
        cameraTapZoomPoint.request.x = 0.8;
        cameraTapZoomPoint.request.y = 0.7;
        camera_set_tap_zoom_point_client.call(cameraTapZoomPoint);
        sleep(5);
        break;
      }
      case 'g': {
        CameraSetZoomPara cameraSetZoomPara;
        cameraSetZoomPara.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_1);
        cameraSetZoomPara.request.factor = 5;
        camera_set_zoom_para_client.call(cameraSetZoomPara);
        sleep(4);
        cameraSetZoomPara.request.factor = 10;
        camera_set_zoom_para_client.call(cameraSetZoomPara);
        sleep(4);
        CameraZoomCtrl cameraZoomCtrl;
        cameraZoomCtrl.request.start_stop = 1;
        cameraZoomCtrl.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_1);
        cameraZoomCtrl.request.direction = static_cast<uint8_t>(dji_osdk_ros::ZoomDirection::ZOOM_OUT);
        cameraZoomCtrl.request.speed = static_cast<uint8_t>(dji_osdk_ros::ZoomSpeed::FASTEST);
        camera_task_zoom_ctrl_client.call(cameraZoomCtrl);
        sleep(8);
        cameraZoomCtrl.request.start_stop = 0;
        cameraZoomCtrl.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_1);
        camera_task_zoom_ctrl_client.call(cameraZoomCtrl);
        break;
      }
      case 'm': {
        while (1){
          std::cout << std::endl;
          std::cout
          << "| Available commands                 |\n"
          << "| [w] Pitch ++                       |\n"
          << "| [s] Pitch --             Press     |\n"
          << "| [a] Roll ++                p       |\n"
          << "| [d] Roll --               to       |\n"
          << "| [z] Yaw ++               exit      |\n"
          << "| [x] Yaw --                         |\n"
          << "| [c] Choose Value                   |\n";
          std::cin >> inChar;
          //inChar = getch();
          //a = cbreak();
          switch (inChar)
          {
          case 'w' :
            gpitch +=10;
            break;
          case 's' :
            gpitch -=10;
            break;
          case 'a' :
            groll +=10;
            break;
          case 'd' :
            groll -=10;
            break;
          case 'z' :
            gyaw +=10;
            break;
          case 'x' :
            gyaw -=10;
            break;
          case 'c' :
            std::cout << "Enter Pitch Roll Yaw Value";
            std::cin >> gpitch, groll, gyaw;

            break;
          
          default:
            break;
          }
          if (inChar=='p'){
            break;
          }
          
          GimbalAction gimbalAction;
          gimbalAction.request.is_reset = false;
          gimbalAction.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
          gimbalAction.request.rotationMode = 0;
          gimbalAction.request.pitch = gpitch;
          gimbalAction.request.roll = groll;
          gimbalAction.request.yaw = gyaw;
          gimbalAction.request.time = 0.5;
          gimbal_control_client.call(gimbalAction);

          gimbalv.x=gpitch;
          gimbalv.y=groll;
          gimbalv.z=gyaw;
          gimbal_angle_publisher_.publish(gimbalv);
          ros::spinOnce();
        }
        break;
      }

      case 'n': {
        GimbalAction gimbalAction;
        gimbalAction.request.is_reset = true;
        gimbalAction.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        gimbal_control_client.call(gimbalAction);
        break;
      }

      case 'q': {
        std::cout << "Quit now ..." << std::endl;
        setupCameraStream_.request.cameraType = setupCameraStream_.request.MAIN_CAM;
        setupCameraStream_.request.start = 0;
        setup_camera_stream_client.call(setupCameraStream_);
        return 0;
      }

      default:
        break;
    }
  }
}