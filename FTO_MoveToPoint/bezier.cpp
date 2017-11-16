#include "bezier.h"
#include <cstring>
#include "stdafx.h"

bezier::vec2 bezier::getBezierPoint( bezier::vec2* points, int numPoints, float t) {
    //LOG_WARNING(cString::Format("bezier::vec_x %f, vec_y %f, numPoints %d, t %f",vec2[0].x, vec2[0].y, numPoints, t));
    bezier::vec2* tmp = new bezier::vec2[numPoints];
    memcpy(tmp, points, numPoints * sizeof(vec2));
    int i = numPoints - 1;
    while (i > 0) {
        for (int k = 0; k < i; k++)
            tmp[k] = tmp[k] + ( tmp[k+1] - tmp[k] ) * t;
        i--;
    }
    vec2 answer = tmp[0];
    delete[] tmp;
    return answer;
}
