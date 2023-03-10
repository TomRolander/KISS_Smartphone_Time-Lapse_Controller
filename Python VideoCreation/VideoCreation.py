"""
/**************************************************************************
  VideoCreation
    The purpose of this Python program is to input a folder of image
  files (JPG), watermark the images (ImageMagick), rename the files, 
  and create a video of the watermarked files (FFMPEG).

  Original Code:  2022-10-08

  Tom Rolander, MSEE
  Mentor, Circuit Design, Software, and 3D Printing
  Miller Library, Fabrication Lab
  Hopkins Marine Station, Stanford University,
  120 Ocean View Blvd, Pacific Grove, CA 93950
  +1 831.915.9526 | rolander@stanford.edu

 **************************************************************************/

 launch.json additional values

                "--inputdir", ".\\Input",
                "--outputdir", ".\\Output",
                "--fps", "16",
                
To Do List
    - compute average time between pics
    - add optional string to -annotate
            -annotate 90x90+150+30 %[exif:DateTimeOriginal]_fps" + str(frames_per_second).zfill(2) + " -fill

"""
Version = "Ver 0.3"
RevisionDate = "2023-03-03"

import sys
import os
import time
import subprocess
import argparse
import time
import exifread
import datetime
import array
import statistics
import platform

from scipy import stats

from datetime import datetime
from datetime import timedelta

NoneType = type(None)

minutes_elapsed = array.array('f')

separator = "\\"

print ("VideoCreation",Version, RevisionDate)
if platform.system() == "Windows":
    print("Running on Windows")
else:
    print("Running on Mac")
    separator = "/"

parser = argparse.ArgumentParser("Video Creation from Photos")
parser.add_argument('--verbose', action='store_true', help="Show more context")
parser.add_argument('--inputdir', type=str, required=False)
parser.add_argument('--outputdir', type=str, required=False)
parser.add_argument('--nextimagenumber', type=int, required=False)
parser.add_argument('--fps', type=int, required=False)
parser.add_argument('--crf', type=int, required=False, help="CRF range 0-50 default is 23")
parser.add_argument('--numberofpoints', type=int, required=False)
parser.add_argument('--videocreate', required=False, choices=('True','False'))
args = parser.parse_args()

if args.verbose:
    print(f"Create MP4 time lapse video from a folder of JPGs")

now = datetime.now()
start = time.time()

start_time = now.strftime("%H:%M:%S")
print("Start Time =", start_time)

if type(args.inputdir) is NoneType:
    input_dir_name = os.getcwd() + separator + "Input"
else:
    input_dir_name = args.inputdir
print("Input Dir = ", end ="")
print(input_dir_name)

if type(args.outputdir) is NoneType:
    output_dir_name = os.getcwd() + separator + "Output"
    watermarked_dir_name = output_dir_name + separator + "Watermarked"
    videos_dir_name = output_dir_name + separator + "Videos"
else:
    output_dir_name = args.outputdir
    watermarked_dir_name = args.outputdir + separator + "Watermarked"
    videos_dir_name = args.outputdir + separator + "Videos"
print("Output Dir Watermarked = ", end ="")
print(watermarked_dir_name)
print("Output Dir Videos = ", end ="")
print(videos_dir_name)

if type(args.nextimagenumber) is NoneType:
    isExist = os.path.exists(watermarked_dir_name + separator + "0")
    if (isExist == False):
        base = 0
    else:
        base = 0
        # Iterate directory
        for path in os.listdir(watermarked_dir_name + separator + "0"):
            # check if current path is a file
            if os.path.isfile(os.path.join(watermarked_dir_name + separator + "0", path)):
                base += 1
else:
    base = args.nextimagenumber - 1
print("Next image number = ", end ="")
print(base+1)

if type(args.fps) is NoneType:
    frames_per_second = 16
else:
    frames_per_second = args.fps
print("Frames per second = ", end ="")
print(frames_per_second)

if type(args.crf) is NoneType:
    constant_rate_factor = 23
else:
    constant_rate_factor = args.crf
print("Constant rate factor = ", end ="")
print(constant_rate_factor)

if type(args.numberofpoints) is NoneType:
    number_of_points = 10
else:
    number_of_points = args.numberofpoints
print("Number of points = ", end ="")
print(number_of_points)

isExist = os.path.exists(input_dir_name)
if (isExist == False):
    print("Input Dir does not exist", input_dir_name)
    exit(1)

count = 0

# Iterate directory

# Get the file count
for path in os.listdir(input_dir_name):
    # check if current path is a file
    #print(path)
    if (path.endswith('JPG') == False) and (path.endswith('jpg') == False) and (path.endswith('jpeg') == False):
        continue        
    if os.path.isfile(os.path.join(input_dir_name, path)):
        count += 1
        #print(os.path.join(input_dir_name, path))
print('Input file count:', count)

if count % number_of_points != 0:
    print("Your number of input files is not a multiple of your number of points!")
    exit(1)

havepreviousvalue = False
index = 0
nmb = 0

print('Rename files to EXIF Date Taken')
for path in os.listdir(input_dir_name):
    # check if current path is a file
    #print(path)
    if (path.endswith('JPG') == False) and (path.endswith('jpg') == False) and (path.endswith('jpeg') == False):
        continue
    with open(os.path.join(input_dir_name, path), "rb") as image:
        exif = exifread.process_file(image)
        dt = str(exif['EXIF DateTimeOriginal']) #get 'Date Taken' from JPG
        ds = time.strptime(dt, '%Y:%m:%d %H:%M:%S')

        if ((number_of_points == 1) or ((index % number_of_points) == 0)):
            dd = datetime.strptime(dt, '%Y:%m:%d %H:%M:%S')
            if havepreviousvalue == False:
                havepreviousvalue = True
            else:
                delta = dd - dd_previous
                minutes_elapsed.append(delta.total_seconds()/60)
                #print(f"Time difference is {minutes_elapsed[nmb]} minutes")
                nmb = nmb + 1
            dd_previous = dd
        index = index + 1

        nt = time.strftime("%Y-%m-%d_%H-%M-%S",ds)
        newname = nt + ".JPG"
        image.close()
        #print("Rename " + os.path.join(input_dir_name,path) + " to " + os.path.join(input_dir_name,newname))
        os.rename(os.path.join(input_dir_name,path), os.path.join(input_dir_name,newname))

if (index > 0):
    print("Mean minutes between photos = ", end ="")
    print("{:.1f}".format(statistics.mean(minutes_elapsed)))
    print("Trim mean minutes between photos of 0.025 = ", end ="")
    print("{:.1f}".format(stats.trim_mean(minutes_elapsed, 0.025)))
    print("Trim mean minutes between photos of 0.05 = ", end ="")
    print("{:.1f}".format(stats.trim_mean(minutes_elapsed, 0.05)))
    print("Trim mean minutes between photos of 0.10 = ", end ="")
    print("{:.1f}".format(stats.trim_mean(minutes_elapsed, 0.10)))
    print("Trim mean minutes between photos of 0.15 = ", end ="")
    print("{:.1f}".format(stats.trim_mean(minutes_elapsed, 0.15)))
    print("Trim mean minutes between photos of 0.20 = ", end ="")
    print("{:.1f}".format(stats.trim_mean(minutes_elapsed, 0.20)))
    print("Trim mean minutes between photos of 0.25 = ", end ="")
    print("{:.1f}".format(stats.trim_mean(minutes_elapsed, 0.25)))
    hours_per_second_of_video = "{:.1f}".format((frames_per_second * stats.trim_mean(minutes_elapsed, 0.10) ) / 60)
else:
    hours_per_second_of_video = "0.0"

print("1 Sec Video = ", end ="")
print(hours_per_second_of_video, end ="")
print(" Hour Real Time")
hours_per_second_of_video = "1Sec" + hours_per_second_of_video + "Hr"

if type(args.videocreate) is NoneType or args.videocreate == True:
    print("Create MP4 video")
    bVideocreate = True
else:
    print("Only watermark files, no video will be created")
    bVideocreate = False

isExist = os.path.exists(output_dir_name)
if (isExist == False):
    print("Creating Output Dir ", output_dir_name)
    os.mkdir(output_dir_name)

isExist = os.path.exists(watermarked_dir_name)
if (isExist == False):
    print("Creating Output Watermarked Dir ", watermarked_dir_name)
    os.mkdir(watermarked_dir_name)

if bVideocreate == True:
    isExist = os.path.exists(videos_dir_name)
    if (isExist == False):
        print("Creating Output Videos Dir ", videos_dir_name)
        os.mkdir(videos_dir_name)

# Get list of all files only in the given directory
##list_of_files = filter( lambda x: os.path.isfile(os.path.join(input_dir_name, x)),
##                        os.listdir(input_dir_name) )
# Sort list of files based on last modification time in ascending order
##list_of_files = sorted( list_of_files,
##                        key = lambda x: os.path.getmtime(os.path.join(input_dir_name, x))
##                        )

list_of_files = sorted( filter( lambda x: os.path.isfile(os.path.join(input_dir_name, x)),
                        os.listdir(input_dir_name) ) )

for i in range(0, number_of_points):
    #print("\"" + watermarked_dir_name + separator + "" + str(i) + "\"")
    isExist = os.path.exists(watermarked_dir_name + separator + "" + str(i))
    if (isExist == False):
        print("\"" + watermarked_dir_name + separator + "" + str(i) + "\"")
        os.mkdir(watermarked_dir_name + separator + "" + str(i))

index = 0
#base = 0
print("Watermarking photos:")
for file_name in list_of_files:
    if ((index % number_of_points) == 0):
        base += 1
    input_file_path = os.path.join(input_dir_name, file_name)
    #timestamp_str = time.strftime(  '%m/%d/%Y %H:%M:%S',
    #                            time.gmtime(os.path.getmtime(input_file_path))) 
    print(str(index+1).zfill(6), str((index % number_of_points)), str(base).zfill(6), file_name) 
    output_file_path =  watermarked_dir_name + separator + "" + str((index % number_of_points)) + separator + "" + str(base).zfill(6) + ".jpg"
#    output_file_path =  watermarked_dir_name + separator + "" + str((index % number_of_points)) + separator + "" + file_name
    #print(output_file_path)

    os.system("magick convert " + "\"" + input_file_path + "\"" + " -quiet -gravity northwest -font Arial-bold -pointsize 144 -fill black -annotate 90x90+150+30 %[exif:DateTimeOriginal] -fill white -annotate 90x90+155+35 %[exif:DateTimeOriginal] \"" + output_file_path + "\"")
    #print ("Done")
    index += 1

if bVideocreate == True:
    for i in range(0, number_of_points):
        #print (output_dir_name + separator + "" + "P" + str(i) + "_%05d")
        #print (videos_dir_name + separator + "" + "VideoPoint_" + str(i) + ".mp4")
        print ("ffmpeg -loglevel error -r " + str(frames_per_second) + " -f image2 -s 1920x1080 -i " + "\"" + watermarked_dir_name + separator + "" + str(i) + separator + ""+ "%06d.jpg" + "\"" + " -vcodec libx264 -crf " + str(constant_rate_factor) + " -pix_fmt yuv420p -y " + "\"" + videos_dir_name + separator + "" + "Pt" + str(i) + "_fps" + str(frames_per_second).zfill(2) + "_crf" + str(constant_rate_factor).zfill(2) + "_" + hours_per_second_of_video + ".mp4" + "\"")
        os.system("ffmpeg -loglevel error -r " + str(frames_per_second) + " -f image2 -s 1920x1080 -i " + "\"" + watermarked_dir_name + separator + "" + str(i) + separator + ""+ "%06d.jpg" + "\"" + " -vcodec libx264 -crf " + str(constant_rate_factor) + " -pix_fmt yuv420p -y " + "\"" + videos_dir_name + separator + "" + "Pt" + str(i) + "_fps" + str(frames_per_second).zfill(2) + "_crf" + str(constant_rate_factor).zfill(2) + "_" + hours_per_second_of_video + ".mp4" + "\"")

end_time = now.strftime("%H:%M:%S")
print("End Time =", end_time)
end = time.time()
print("Elapsed time = ", end ="")
#print('{:.1f}'.format(end - start))
td = timedelta(seconds=int(end - start))
print(td)