/**
 * MIT License
 * Copyright (c) 2018 Patrick Geneva @ University of Delaware (Robot Perception & Navigation Group)
 * Copyright (c) 2018 Kevin Eckenhoff @ University of Delaware (Robot Perception & Navigation Group)
 * Copyright (c) 2018 Guoquan Huang @ University of Delaware (Robot Perception & Navigation Group)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#ifndef MSCKF_GRIDER_FAST_H
#define MSCKF_GRIDER_FAST_H


#include <vector>
#include <iostream>
#include <Eigen/Eigen>


#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


class Grider_FAST {

public:


    /**
     * Compare keypoints based on their response value
     * We want to have the keypoints with the highest values!
     * https://stackoverflow.com/a/10910921
     */
    static bool compare_response(cv::KeyPoint first, cv::KeyPoint second) {
        return first.response > second.response;
    }


    /**
     * This function will perform grid extraction using FAST
     * Given a specified grid size, this will try to extract fast features from each grid
     * It will then return the best from each grid
     */
    static void perform_griding(const cv::Mat &img, std::vector<cv::KeyPoint> &pts, int num_features,
                                int grid_x, int grid_y, int threshold, bool nonmaxSuppression) {

        // Calculate the size our extraction boxes should be
        int size_x = img.cols / grid_x;
        int size_y = img.rows / grid_y;

        // Make sure our sizes are not zero
        assert(size_x > 0);
        assert(size_y > 0);

        // We want to have equally distributed features
        auto num_features_grid = (int) (num_features / (grid_x * grid_y)) + 1;

        // Lets loop through each grid and extract features
        for (int x = 0; x < img.cols; x += size_x) {
            for (int y = 0; y < img.rows; y += size_y) {

                // Skip if we are out of bounds
                if (x + size_x > img.cols || y + size_y > img.rows)
                    continue;

                // Calculate where we should be extracting from
                cv::Rect img_roi = cv::Rect(x, y, size_x, size_y);

                // Extract FAST features for this part of the image
                std::vector<cv::KeyPoint> pts_new;
                cv::FAST(img(img_roi), pts_new, threshold, nonmaxSuppression);

                // Now lets get the top number from this
                std::sort(pts_new.begin(), pts_new.end(), Grider_FAST::compare_response);

                // Debug print out the response values
                //for (auto pt : pts_new) {
                //    std::cout << pt.response << std::endl;
                //}
                //std::cout << "======================" << std::endl;

                // Append the "best" ones to our vector
                // Note that we need to "correct" the point u,v since we extracted it in a ROI
                // So we should append the location of that ROI in the image
                for(size_t i=0; i<(size_t)num_features_grid && i<pts_new.size(); i++) {
                    cv::KeyPoint pt_cor = pts_new.at(i);
                    pt_cor.pt.x += (float)x;
                    pt_cor.pt.y += (float)y;
                    pts.push_back(pt_cor);
                }


            }
        }


    }


};


#endif /* MSCKF_GRIDER_FAST_H */