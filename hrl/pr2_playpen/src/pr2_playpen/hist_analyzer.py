#!/usr/bin/env python  
# Calculating and displaying 2D Hue-Saturation histogram of a color image
import roslib
roslib.load_manifest('opencv2')
import sys
import cv


cv.NamedWindow("avg_noise", cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow("back_modified", cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow("back_modified2", cv.CV_WINDOW_AUTOSIZE)

class HistAnalyzer:

    def __init__(self, background_noise, mask):
        self.background_noise = background_noise
        self.h_bins = 30
        self.s_bins = 32
        self.h_ranges = [0, 180]
        self.s_ranges = [0, 255]
        self.ranges = [self.h_ranges, self.s_ranges]
        self.hist = None
        self.mask = mask
        self.avg_noise = None

    def calc_hist(self):
        self.hist = cv.CreateHist([self.h_bins, self.s_bins], cv.CV_HIST_ARRAY, self.ranges, 1)
        hsv = cv.CreateImage(cv.GetSize(self.background_noise[0]), 8, 3)
        h_plane = cv.CreateMat(self.background_noise[0].height, self.background_noise[0].width, cv.CV_8UC1)
        s_plane = cv.CreateMat(self.background_noise[0].height, self.background_noise[0].width, cv.CV_8UC1)
        for i in xrange(len(self.background_noise)):
            cv.CvtColor(self.background_noise[i], hsv, cv.CV_BGR2HSV)
            cv.Split(hsv, h_plane, s_plane, None, None)            
            planes = [h_plane, s_plane]#, s_plane, v_plane]
            cv.CalcHist([cv.GetImage(i) for i in planes], self.hist, True, self.mask)            
        #cv.NormalizeHist(self.hist, 1.0)

    def check_for_hist(self):
        if self.hist == None:
            print "YOU CAN'T CALCULATE NOISE WITH HIST MODEL OF TABLETOP"
            exit

    def calc_noise(self):
        self.check_for_hist()
        self.avg_noise = cv.CreateImage(cv.GetSize(self.background_noise[0]), 8, 1)
        cv.Zero(self.avg_noise)

        for i in xrange(len(self.background_noise)-1):
            back_proj_img1, hist1 = self.back_project_hs(self.background_noise[i])
            back_proj_img2, hist2 = self.back_project_hs(self.background_noise[i+1])

            scratch = cv.CreateImage(cv.GetSize(back_proj_img2), 8, 1)
            scratch2 = cv.CreateImage(cv.GetSize(back_proj_img2), 8, 1)
            
            # do something clever with ands ors and diffs 
            cv.Zero(scratch)
            cv.Zero(scratch2)
            cv.Sub(back_proj_img2, back_proj_img1, scratch2) #noise, but includes object if failed, 

            cv.Sub(scratch2, self.avg_noise, scratch)            
            cv.Or(self.avg_noise, scratch2, self.avg_noise)
            cv.WaitKey(100)


    def back_project_hs(self, img):
        self.check_for_hist()
        hsv = cv.CreateImage(cv.GetSize(img), 8, 3)
        scratch = cv.CreateImage(cv.GetSize(img), 8, 1)
        back_proj_img = cv.CreateImage(cv.GetSize(img), 8, 1)
        cv.CvtColor(img, hsv, cv.CV_BGR2HSV)
        h_plane_img = cv.CreateImage(cv.GetSize(img), 8, 1)
        s_plane_img = cv.CreateImage(cv.GetSize(img), 8, 1)
        cv.Split(hsv, h_plane_img, s_plane_img, None, None)            
        cv.CalcBackProject([h_plane_img, s_plane_img], back_proj_img, self.hist)
        cv.MorphologyEx(back_proj_img, back_proj_img, None, None, cv.CV_MOP_OPEN, 1)
        cv.MorphologyEx(back_proj_img, back_proj_img, None, None, cv.CV_MOP_CLOSE, 2)    
        cv.Threshold(back_proj_img, back_proj_img, 250, 255, cv.CV_THRESH_BINARY)

        return back_proj_img, self.hist

if __name__ == '__main__':
    folder = sys.argv[1]+'/background_noise/'
    background_noise = []

    cv.NamedWindow("Source", cv.CV_WINDOW_AUTOSIZE)
    cv.NamedWindow("final", cv.CV_WINDOW_AUTOSIZE)

    for i in xrange(130):
        background_noise.append(cv.LoadImage(folder+'file'+str(i).zfill(3)+'.png'))
    mask = cv.LoadImage(sys.argv[2], 0)

    ha = HistAnalyzer(background_noise, mask)
    ha.calc_hist()
    ha.calc_noise()

    for i in xrange(9):
        for j in xrange(100):
            print sys.argv[1]+'/object'+str(i).zfill(3)+'_try'+str(j).zfill(3)+'_after_pr2.png'
            try:
                img = cv.LoadImageM(sys.argv[1]+'/object'+str(i).zfill(3)+'_try'+str(j).zfill(3)+'_after_pr2.png')
                cv.ShowImage("Source", img)

                back_proj_img, hist1 = ha.back_project_hs(img)
                back_proj_img2, hist2 = ha.back_project_hs(ha.background_noise[0])

                scratch = cv.CreateImage(cv.GetSize(back_proj_img2), 8, 1)
                scratch2 = cv.CreateImage(cv.GetSize(back_proj_img2), 8, 1)

                # do something clever with ands ors and diffs 
                cv.Zero(scratch)
                cv.Zero(scratch2)

                #cv.Sub(back_proj_img, back_proj_img2, scratch2) #opposite noise, but excludes object 
                cv.Sub(back_proj_img2, back_proj_img, scratch2) #noise, but includes object if failed, 
                cv.Sub(scratch2, ha.avg_noise, scratch) 

                cv.ShowImage("final", scratch)

                #cv.ShowImage("back_projection", back_proj_img2)
                cv.WaitKey(33)

                cv.Scale(back_proj_img, back_proj_img, 1/255.0)
                print "here's the sum :", cv.Sum(scratch)
            except:
                print "no file like that, probably outside of index range"

