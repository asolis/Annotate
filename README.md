## Annotate is a multiple object ground truth annotation tool using polygons. 

The Annotate tool allows you to create your video sequence annotations using representations such as, 
rectangles, rotated rectangles and polygons. It generates a text file with the annotations
for each frame following a simple CSV format. 

Among the possible applications Annotate could be useful for creating ground truth for:

1. Pedestrian, people or re-identification datasets
2. Upper body, face or other parts datasets
3. Object detection datasets
4. Object tracking datasets

In general, any project where a polygonal representation of your target is needed, Annotate will
help to create it. 

The Annotate tool can also provide a prediction for target positions to help annotating large datasets. 
Currently, it uses our [sKCF](https://github.com/asolis/vivaTracker) tracking algorithm to 
predict the target's new location.  

 
For more detailed information and examples of how to use reefer to the project's  [wiki](https://github.com/asolis/Annotate/wiki).
