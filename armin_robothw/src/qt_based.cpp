#include <qapplication.h>
#include "window_first.h"

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

int main( int argc, char **argv )
{
    QApplication a( argc, argv );

    ros::init(argc, argv, "gui_state_pub");

    WindowFirst hello{};

    hello.show();
    return a.exec();
}

