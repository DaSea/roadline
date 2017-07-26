/*
 * Copyright (C) 2016-01-25 Dasea <dhf0214@126.com>
 *
 * Description: 基本类型定义
 */

#ifndef __BASE_GEOMETRY_H__
#define __BASE_GEOMETRY_H__


#define LOG_TAG "kq_road"

//#define KQ_ROAD_DEBUG

#define INFO_OUT(...)
#define WARN_OUT(...)
#define ERROR_OUT(...)
#define DEBUG_OUT(...)

#ifndef NULL
#define NULL ((void*)0)
#endif

#define LINE_GAP (5)

#ifndef PI
#define PI (3.1415926535897)
#endif

// 偏角，固定为90度，即π/2
#define BORDER_ANGLE  (PI/2)
//根据角度获得弧度
#define   RADIAN(a) (a*PI/180.0)
//根据弧度获得角度
#define   ANGLE(r)  (180.0*r/PI)

// 左转
#define DIRECTION_TYPE_LEFT     (-1)
// 不转
#define DIRECTION_TYPE_NO       (0)
// 右转
#define DIRECTION_TYPE_RIGHT    (1)

// 直线的最小长度,线元
#define LEASET_LINE_LEN         (0.1)

struct Point3D {
    Point3D() {
        x=0;
        y=0;
        z=0;
        azimuth = 0;
    }

    Point3D(double dx,double dy,double dz, double dazimuth=0) {
        x=dx;
        y=dy;
        z=dz;
        azimuth = dazimuth;
    }

    Point3D(const Point3D& point) {
        x=point.x;
        y=point.y;
        z=point.z;
        azimuth = point.azimuth;
    }

    Point3D& operator=(const Point3D& point) {
        if (this == &point) return *this;
        x = point.x;
        y = point.y;
        z = point.z;
        azimuth = point.azimuth;
        return *this;
    }

    bool operator==(const Point3D& Point)
    {
        if ((x-Point.x)<=0.00001&&(y-Point.y)<=0.00001)
            return true;
        return false;
    }

    double x;
    double y;
    double z;
    double azimuth; // 方位角,弧度
};

#endif
