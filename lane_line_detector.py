#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import numpy as np
import cv2


class LaneLineDetector(object):
    def __init__(self, img, record_process=False, tolerance=1e-6):
        assert img.ndim == 3

        self.__record_process = record_process
        self.__tolerance = tolerance

        if self.__record_process:
            print("img of type BGR")

        self.img = img
        self.gray_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        self.HEIGHT = self.img.shape[0]
        self.WIDTH = self.img.shape[1]

    def canny_edge(self, gray_img, kernel_size=5, weak_val=50, strong_val=150):
        blurred_img = cv2.GaussianBlur(gray_img, (kernel_size, kernel_size), 0)
        canny_edge = cv2.Canny(blurred_img, weak_val, strong_val)
        return canny_edge

    def segment_regions(self, img, polygons):
        if self.__record_process:
            print("size of polygons {}".format(len(polygons)))

        mask = np.zeros_like(img)
        cv2.fillPoly(mask, polygons, 255)

        segment = cv2.bitwise_and(img, mask)
        return segment

    def hough_lines(
        self,
        img,
        rho=2,
        theta=np.pi / 180,
        threshold=100,
        min_line_len=100,
        max_line_gap=50,
    ):
        lines = cv2.HoughLinesP(
            img,
            rho,
            theta,
            threshold,
            np.array([]),
            minLineLength=min_line_len,
            maxLineGap=max_line_gap,
        )

        if lines is not None:
            return lines
        else:
            return []

    def estimate_left_right_lanes_parameters(self, lines):
        left_lane = []
        right_lane = []
        for line in lines:
            for x1, x2, y1, y2 in line:
                p1 = (x1, y1)
                p2 = (x2, y2)
                parameters = np.polyfit(p1, p2, 1)
                slope, intercept = parameters
                if slope < 0:
                    left_lane.append((slope, intercept))
                else:
                    right_lane.append((slope, intercept))

        left_avg = None
        right_avg = None

        if len(left_lane) > 0:
            left_avg = np.average(left_lane, axis=0)

        if len(right_lane) > 0:
            right_avg = np.average(right_lane, axis=0)

        return [left_avg, right_avg]

    def estimate_left_right_lanes_points(
        self, img, left_right_parameters, y_offset=200
    ):
        height = img.shape[0]
        width = img.shape[1]

        def __estimate_lanes_points(parameters):
            slope, intercept = parameters
            slope = np.sign(slope) * max(abs(slope), self.__tolerance)

            y1 = height - 1
            y2 = int(y1 - y_offset)
            y2 = max(0, y2)

            x1 = int((y1 - intercept) / slope)

            x2 = int((y2 - intercept) / slope)

            return [(x1, y1), (x2, y2)]

        left_right_lane_points = []
        for parameters in left_right_parameters:
            if parameters is not None:
                left_right_lane_points.append(__estimate_lanes_points(parameters))

        return left_right_lane_points

    def estimate_lanes(
        self,
        polygons,
        kernel_size=5,
        weak_val=50,
        strong_val=150,
        y_offset=200
    ):
        gray_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        edged_img = self.canny_edge(gray_img, kernel_size, weak_val, strong_val)
        segmented_img = self.segment_regions(edged_img, polygons)
        rough_lines = self.hough_lines(img=segmented_img)
        left_right_lanes_parameters = self.estimate_left_right_lanes_parameters(
            rough_lines
        )
        left_right_lane_points = self.estimate_left_right_lanes_points(
            self.img, left_right_lanes_parameters, y_offset
        )

        if self.__record_process:
            self.gray_img = gray_img
            self.edged_img = edged_img
            self.segmented_img = segmented_img
            self.rough_lines = rough_lines

        return left_right_lane_points

    def draw_lines(self, img, lane_lines, color=(0, 0, 255), thickness=3):
        visualized_img = np.copy(img)

        for line in lane_lines:
            p1, p2 = line
            cv2.line(visualized_img, p1, p2, color, thickness)
        return visualized_img

    def draw_overlayed_lines(
        self, img, lane_lines, color=(139, 0, 139), thickness=3, α=0.8, β=1.0, γ=0
    ):
        visualized_img = np.zeros_like(img)
        visualized_img = self.draw_lines(visualized_img, lane_lines, color, thickness)
        visualized_img = cv2.addWeighted(self.img, α, visualized_img, β, γ)

        return visualized_img
