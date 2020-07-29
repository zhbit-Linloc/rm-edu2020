//#include "opencv2/opencv.hpp"
//#include "threadcontrol.h"
//#include <thread>
//#include "base.h"

//int main(){
//    // 开启相关线程
//    threadControl ImageControl;
//    // 图像生成线程
//    std::thread produce_task(&threadControl::ImageProduce, &ImageControl);  // & == std::ref()
//    // 图像处理线程
//    std::thread process_task(&threadControl::ImageProcess, &ImageControl);
//#ifdef VIDEOSAVE
//    std::thread write_task(&threadControl::ImageWrite, &ImageControl);
//#endif
//    produce_task.join();
//    process_task.join();
//#ifdef VIDEOSAVE
//    write_task.join();
//#endif
//    return 1;
//}
#include "threadpool.h"
#include "threadtask.h"
#include "base.h"

int main(void)
{
    threadPool pool(3);
    threadTask task;
    pool.append(std::bind(&threadTask::ImageProduce, &task));
    pool.append(std::bind(&threadTask::ImageProcess, &task));
#ifdef VIDEOSAVE
    pool.append(std::bind(&threadTask::ImageWrite, &task));
#endif
    return 1;
}


