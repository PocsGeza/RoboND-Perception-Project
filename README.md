## Project: Perception Pick & Place

#### Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

Here it is :)

### Exercises 1, 2 and 3 in one place

`project_template.py` contains all the code needed to satisfy the conditions of exercises 1 to 3. 

[This is my script](https://github.com/PocsGeza/RoboND-Perception-Project/blob/master/pr2_robot/scripts/project_template.py)

#### 1. Implementing RANSAC plane fitting implemented.

The pipeline is implemented in this order:

 * Image
 * Outlier Filter
 * Voxel Grid Downsampling
 * Passthrough over Z-Axis
 * Passthrough over X-Axis
 * RANSAC PLANE Filter
 * Done
          
Python has it's good sides but when you get to pipelines dynamic typing starts to loose it's appeal. At multiple points I felt it equally probable for an object to have type PointCloud2 or type ColoredDinosaurEgg.
   
The steps where quite strait forward. I got a lot of help from the forum on fine tuning the parameters.

This is what i looks like after the pipeline:


![After Pipeline_1](https://github.com/PocsGeza/RoboND-Perception-Project/raw/master/images/pipeline1.png)

#### 2. Clustering for segmentation.  

I used the code example from the lectures for Euclidean Clustering. The values where established by a lot of trial and error and forum.

Here is what it looked like:


![After Pipeline_2](https://github.com/PocsGeza/RoboND-Perception-Project/raw/master/images/pipeline2.png)

#### 3. Features extraction. SVM training. Object recognition.

SVM was created with the following settings:

* Pictures: 500

* Bins (Color & Normals): 64

* RGB vs HST: HST

* Total Features: 4000

Results of the training:


![Figure_1](https://github.com/PocsGeza/RoboND-Perception-Project/raw/master/images/figure_1.png)
![Figure_2](https://github.com/PocsGeza/RoboND-Perception-Project/raw/master/images/figure_2.png)
![Training results](https://github.com/PocsGeza/RoboND-Perception-Project/raw/master/images/training.png)


Here is the object and their markers:

![After Pipeline3](https://github.com/PocsGeza/RoboND-Perception-Project/raw/master/images/pipeline3.png)


### Pick and Place

#### 1. Perform object recognition for each  world. Read in respective list (`pick_list_*.yaml`). Construct the messages and output them to `.yaml` format.

The results:

World 1: 

![World1](https://github.com/PocsGeza/RoboND-Perception-Project/raw/master/images/world_1.png)

[YAML_World_1](https://raw.githubusercontent.com/PocsGeza/RoboND-Perception-Project/master/pr2_robot/scripts/output_1.yaml)

World 2:

![World2](https://github.com/PocsGeza/RoboND-Perception-Project/raw/master/images/world_2.png)

[YAML_World_2](https://raw.githubusercontent.com/PocsGeza/RoboND-Perception-Project/master/pr2_robot/scripts/output_2.yaml)

World 3:

![World3](https://github.com/PocsGeza/RoboND-Perception-Project/raw/master/images/world_3.png)

[YAML_World_3](https://raw.githubusercontent.com/PocsGeza/RoboND-Perception-Project/master/pr2_robot/scripts/output_3.yaml)


At the end of it all:

This was my fist contact with object recognition. The concepts where not that advanced but it was not a step by step process. It was more like a leap by leap process. 
I am 2 months late with this project. I lost 6 weeks because I was let go from my previous company and I had to finish all my work projects before leaving. I was not fired but the client decided to end the project I was on. I could have stayed for a new project but that would have meant a 2 year commitment to the position (I am taking the nano degree get out of the position). So after a month of last minute tasks I opted for temporary unemployment. 

The project more or less works. I haven't learned much. Given the close deadline my goal was to pass the submission.

The bellow image sums up my impresion of the job I managed to do.

![Tape and more tape](http://hvac-hacks.com/wp-content/uploads/2014/03/223.hvac-hacks.com.5323a4c9dff36.jpg)
