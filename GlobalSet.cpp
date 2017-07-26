/*
 * Copyright (C) 2016-01-27 Dasea <dhf0214@126.com>
 *
 * Description: 全局设置
 */
#include "GlobalSet.h"
#include "BaseGeometry.h"

// static
double GlobalSet::_circleLineGap = LINE_GAP;
double GlobalSet::_curveLineGap = LINE_GAP;

void GlobalSet::SetCircleLineGap(double lineGap){
    _circleLineGap = lineGap;
}

void GlobalSet::SetCurveLineGap(double lineGap){
    _curveLineGap = lineGap;
}

double GlobalSet::GetCircleLineGap(){
    return (_circleLineGap);
}

double GlobalSet::GetCurveLineGap(){
    return (_curveLineGap);
}
