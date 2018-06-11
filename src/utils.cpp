// Copyright (C) 2018 Yipeng Li
// 
// This file is part of vo.
// 
// vo is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 2 of the License, or
// (at your option) any later version.
// 
// vo is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with vo.  If not, see <http://www.gnu.org/licenses/>.
// 

#include <utils.h>

namespace image_utils {

    void halfPyr(const cv::Mat& src_, cv::Mat& dst_)
    {
        int width = src_.cols;
        int height = src_.rows;

        int h_width = (int)(width/2);
        int h_height = (int)(height/2);

        dst_ = cv::Mat(h_height, h_width, CV_8UC1);
        // pyramid down
        const int s  = src_.step.p[0];
        uint8_t* t = (uint8_t*)src_.data;
        uint8_t* b = t + s;
        uint8_t* e = t + s*h_height*2;
        uint8_t* ptr = (uint8_t*)dst_.data;

        while (b < e)
        {
            for (int j=0; j<h_width; j++)
            {
                *ptr = static_cast<uint8_t>(uint16_t(t[0]+t[1]+b[0]+b[1])/4);
                ptr++;
                t += 2;
                b += 2;
            }
            t += s;
            b += s;
        }
    }

    void createImgPyr(const cv::Mat& src_, std::vector<cv::Mat>& dst_, int level_)
    {
        dst_.resize(level_);
        dst_[0] = src_;
        for (int i=1; i<level_; i++)
        {
            cv::Mat img_i;
            halfPyr(dst_[i-1], img_i);
            dst_[i] = img_i;
        }
    }

    float shiTomasiScore(const cv::Mat& src_, int u_, int v_)
    {
        assert(src_.type() == CV_8UC1);

        float dxx = 0.0f;
        float dyy = 0.0f;
        float dxy = 0.0f;
        const int w = 4;
        const int dw = 2*w;
        const int a = dw*dw;
        const int x_min = u_-w;
        const int x_max = u_+w;
        const int y_min = v_-w;
        const int y_max = v_+w;

        if (x_min<1 || x_max>=src_.cols-1 || y_min<1 || y_max>=src_.rows-1)
            return 0.0f;

        const int s = src_.step.p[0];
        for (int y=y_min; y<y_max; ++y)
        {
            const uint8_t* pl = src_.data + s*y + x_min - 1;
            const uint8_t* pr = src_.data + s*y + x_min + 1;
            const uint8_t* pt = src_.data + s*(y-1) + x_min;
            const uint8_t* pb = src_.data + s*(y+1) + x_min;
            for (int x = 0; x<dw; ++x, ++pl, ++pr, ++pt, ++pb)
            {
                float dx = *pr - *pl;
                float dy = *pb - *pt;
                dxx += dx*dx;
                dyy += dy*dy;
                dxy += dx*dy;
            }
        }

        dxx = dxx/(2.0*a);
        dyy = dyy/(2.0*a);
        dxy = dxy/(2.0*a);
        return 0.5*(dxx+dyy-sqrt((dxx+dyy)*(dxx+dyy) - 4*(dxx*dyy-dxy*dxy)));
    }
} // end of namespace image_utils