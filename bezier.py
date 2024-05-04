import numpy as np
from scipy.special import comb
class Bezier:

    


    def bezier_curve(points, nTimes=50):
        
        def bernstein_poly(i, n, t):
            return comb(n, i) * ( t**(n-i) ) * (1 - t)**i
        """
        The Bernstein polynomial of n, i as a function of t
        """

            
        """
        Given a set of control points, return the
        bezier curve defined by the control points.

        points should be a list of lists, or list of tuples
        such as [ [1,1], 
                    [2,3], 
                    [4,5], ..[Xn, Yn] ]
            nTimes is the number of time steps, defaults to 1000

            See http://processingjs.nihongoresources.com/bezierinfo/
        """

        nPoints = len(points)
        xPoints = np.array([p[0] for p in points])
        yPoints = np.array([p[1] for p in points])

        t = np.linspace(0.0, 1.0, nTimes)

        polynomial_array = np.array([ bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])

        xvals = np.dot(xPoints, polynomial_array)
        yvals = np.dot(yPoints, polynomial_array)

        return xvals, yvals

    def calculate_control_points(start, end, offset_ratio=0.3):
        # Convert points to numpy arrays for vector operations
        start = np.array(start)
        end = np.array(end)
        
        # Vector from start to end
        vector = end - start
        length = np.linalg.norm(vector)
        direction = vector / length
        
        # Calculate perpendicular direction
        perp_direction = np.array([-direction[1], direction[0]])

        # Determine control points along perpendicular direction
        mid_point = (start + end) / 2
        offset = perp_direction * length * offset_ratio
        
        # Control points are offset from the midpoint
        control1 = mid_point + offset
        control2 = mid_point - offset
        
        return control1, control2

    