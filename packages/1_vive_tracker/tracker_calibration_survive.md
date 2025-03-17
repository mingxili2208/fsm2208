# how to launch the tracker calibration with sutvive

## initing

1. init the car
2. init the tracker

## launch the calibration app

```shell
cd /home/cityu-fsm-lab-carla/lmx/packages/1_vive_tracker/auto_tracker_app

python3 app.py

```

## operating the car

1. click the "init the tracker" button
2. operating the euler calibration
   1. press T
   2. type the true value of tracker (Relative to the sandbox coordinate system; Counterclockwise is positive)
   3. click confirm if the angle and the value of tracker is corrected
   4. Repeat the above operation to obtain at least eight sets of angle correspondences of three positions.(Do not have positive and negative 180 at the same time)
3. operating the position calibration
   1. press r
   2. after 1 if the recording result is correct, remote the car from X=0 to X=maximum(sandbox coor), then roll back the car,(Make sure the front of the car is facing the far yoz plane/the wall with the door in this room 2208)
   3. when the car back to the starting point, press e to pause the recording and move the car to another line.
   4. Repeat the above steps to obtain at least four paths.
4. save all the data. click the button "save all data"
5. transform the data. click the button "transform" to change page, and click the blue transform button in new page.
6. Copy the calculated T and E matrices to the corresponding code of the vr2sx_pysurvive.py file.


roll=137.76141881463616, yaw=179.9573348003782, pitch=0.18613655837748216

roll=-5.360677735149562, yaw=178.57532959880396, pitch=-0.2646571668358826

yaw roll pith 

y z x 