/**
 * Copyright (c)
 * Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
 * 4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************
 * $Author:: schoen $   $Date:: 2016-01-29 #$
 **********************************************************************/

#ifndef ROADSIGN_H_
#define ROADSIGN_H_

class RoadSign {
public:

	cv::Point3f translation;
	cv::Point3f rotation;
	cv::Point3f z_vec;
	int id;
	tTimeStamp time;
	float distance;
	int index;

	RoadSign() {
		time = 0;
		id = -1;
		distance = 0.0001;
		index = -1;
	}

	virtual ~RoadSign() {
	}

	RoadSign(cv::Point3f translation, cv::Point3f rotation, int id, cv::Point3f z_vec, tTimeStamp time) {
    	this->translation = translation;
    	this->distance = cv::norm(translation);
    	this->rotation = rotation;
    	this->id = id;
    	this->time = time;
    	this->z_vec = z_vec;
    	index = -1;
	}

	bool Compare(RoadSign sign, tFloat32 maxDistanceDiff, tTimeStamp maxTimeDiff) {
		if(this->id != sign.id) {
			return false;
		}

		if(abs(cv::norm(translation) - cv::norm(sign.translation)) > maxDistanceDiff) {
			return false;
		}

		if(abs(time - sign.time) > maxTimeDiff) {
			return false;
		}

		return false;
	}
};

#endif /* ROADSIGN_H_ */
