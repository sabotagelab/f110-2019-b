
'''
2.54,2.41,0.0
3.95,3.11,0.0
5.14,3.63,0.0
6.21,4.30,0.0
7.28,5.08,0.0
7.65,6.19,0.0
'''

STEP = 0.1
def do_linear_interpolation(x_start,y_start,xgoal,ygoal):
    steps_count=abs(xgoal-x_start)/STEP
    delta_x= (xgoal-x_start)/steps_count
    delta_y= (ygoal-y_start)/steps_count
    for i in range(int(steps_count)):
        x_start += delta_x
        y_start += delta_y
        print x_start,',',y_start,',',0.0


do_linear_interpolation(2.54,2.41,3.95,3.11)
do_linear_interpolation(3.95,3.11,5.14,3.63)
do_linear_interpolation(5.14,3.63,6.21,4.30)
do_linear_interpolation(6.21,4.30,7.28,5.08)
do_linear_interpolation(7.28,5.08,7.65,6.19)