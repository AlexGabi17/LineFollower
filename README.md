# LineFollower

The final project of the Introduction To Robototics course is implementing a car follower of a black line. For this it were used the following electronical parts: QTR8 light sensors module,
2 DC servo motors with a L293d driver, a LiPo battery. All of them are being controlled by the Arduino Uno.


The error value calculated at every step was mapped from [0, 5000] to the interval of [-255, 255].
The PID values were fine-tuned during the hackathon day at the university were we had the opportunity to test our line followers. After analyzing diffenrent values we finally
got the kp = 10, kd = 15. This were also used in the video below. At its best the line follower finished the track in 26 seconds.

The base speed was set to 200 out of 255. For increased speed the values of PID should be furthermore analyzed and adapted.


[Link Video](https://www.youtube.com/shorts/gHOhgEdUGJ8)
