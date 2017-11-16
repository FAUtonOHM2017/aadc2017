$ Author: hiller $  $ Date 2016-01-06 $

README for creating the system.xml file for ACC
example taken is "UltrasonicACC_greaterRange.xml

value-graph seperated in three different areas:

--> 1) line that stays on value zero, so no speed is applied: here distance [0.0 0.3]

--> 2) parabolically increasing graph until certain value: here distance [0.3 0.95]
       Formula for calculating values: y_2 = a(x_2 - x_a)^2 + y_a (parabolic formula)
       As apex (x_a y_a) = (0.3 0.95) was taken, as graph-point (x_2 y_2) = (0.95 0.5)
       --> calculate parameter 'a' through inserting the known graph-point and the apex;
           afterwards, the remaining supporting points can be calculated
    
--> 3) parabolically decreasing grpah until certain value: here distance [0.95 1.3]
       Formula for calculating values: y_3 = b(x_3 - x_a)^2 + y_a (parabolic formula)
       As apex (x_a y_a) = (1.3 1.0) was taken, as graph-point (x_3 y_3) = (0.95 0.5)
       --> calculate parameter 'b' through inserting the known graph-point and the apex;
           afterwards, the remaining supporting points can be calculated