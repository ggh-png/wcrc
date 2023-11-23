#! /usr/bin/env python3
# -*- coding:utf-8 -*-

class MovingAverageFilter:
    def __init__(self):

        self.window_size = 5
        self.data_list = []

    def _call_(self, new_data):

        tmp = new_data
        
        
        if len(self.data) < self.window_size:
            self.data_list.append(tmp)
        else:
            self.data_list.pop()

        moving_avg = sum(self.data_list) / len(self.data_list)
        return moving_avg

