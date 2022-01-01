# Triangle strips
#
# Usage: python main.py file_of_triangles
#
# You can press ESC in the window to exit.
#
# You'll need Python 3 and must install these packages:
#
#   PyOpenGL, GLFW

#developed by Adam Bayley and Randy Bornstein
import sys, os, math

try: # PyOpenGL
  from OpenGL.GL import *
except:
  print( 'Error: PyOpenGL has not been installed.' )
  sys.exit(0)

try: # GLFW
  import glfw
except:
  print( 'Error: GLFW has not been installed.' )
  sys.exit(0)



# Globals

window = None

windowWidth  = 1000 # window dimensions
windowHeight = 1000

minX = None # range of vertices
maxX = None
minY = None
maxY = None

r  = 0.008 # point radius as fraction of window size

allVerts = [] # all triangle vertices

lastKey = None  # last key pressed

showForwardLinks = True



# Triangle
#
# A Triangle stores its three vertices and pointers to any adjacent triangles.
#
# For debugging, you can set the 'highlight1' and 'highlight2' flags
# of a triangle.  This will cause the triangle to be highlighted when
# it's drawn.


class Triangle(object):

    nextID = 0

    def __init__( self, verts ):

      self.verts   = verts # 3 vertices.  Each is an index into the 'allVerts' global.
      self.adjTris = [] # adjacent triangles

      self.nextTri = None  # next triangle on strip
      self.prevTri = None  # previous triangle on strip
      self.mesh = False
      
      self.highlight1 = False # to cause drawing to highlight this triangle in colour 1
      self.highlight2 = False # to cause drawing to highlight this triangle in colour 2

      self.centroid = ( sum( [allVerts[i][0] for i in self.verts] ) / len(self.verts),
                        sum( [allVerts[i][1] for i in self.verts] ) / len(self.verts) )

      self.id = Triangle.nextID
      Triangle.nextID += 1


    # String representation of this triangle
    
    def __repr__(self):
        return 'tri-%d' % self.id


    # Draw this triangle
    
    def draw(self):

        # Highlight with yellow fill

        if self.highlight1 or self.highlight2:

            if self.highlight1:
                glColor3f( 0.9, 0.9, 0.4 ) # dark yellow
            else:
                glColor3f( 1, 1, 0.8 ) # light yellow

            glBegin( GL_POLYGON )
            for i in self.verts:
                glVertex2f( allVerts[i][0], allVerts[i][1] )
            glEnd()

        # Outline the triangle

        glColor3f( 0, 0, 0 )
        glBegin( GL_LINE_LOOP )
        for i in self.verts:
            glVertex2f( allVerts[i][0], allVerts[i][1] )
        glEnd()


    # Draw edges to next and previous triangle on the strip

    def drawPointers(self):

        if showForwardLinks and self.nextTri:
            glColor3f( 0, 0, 1 )
            drawArrow( self.centroid[0], self.centroid[1], 
                       self.nextTri.centroid[0], self.nextTri.centroid[1] )

        if not showForwardLinks and self.prevTri:
            glColor3f( 1, 0, 0 )
            drawArrow( self.centroid[0], self.centroid[1], 
                       self.prevTri.centroid[0], self.prevTri.centroid[1] )

        if not self.nextTri and not self.prevTri: # no links.  Draw a dot.
            if showForwardLinks:
                glColor3f( 0, 0, 1 )
            else:
                glColor3f( 1, 0, 0 )
            glBegin( GL_POLYGON )
            for i in range(100):
                theta = 3.14159 * i/50.0
                glVertex2f( self.centroid[0] + 0.5 * r * math.cos(theta), self.centroid[1] + 0.5 * r * math.sin(theta) ) 
            glEnd()


    # Determine whether this triangle contains a point
    
    def containsPoint( self, pt ):

        return (turn( allVerts[self.verts[0]], allVerts[self.verts[1]], pt ) == LEFT_TURN and
                turn( allVerts[self.verts[1]], allVerts[self.verts[2]], pt ) == LEFT_TURN and
                turn( allVerts[self.verts[2]], allVerts[self.verts[0]], pt ) == LEFT_TURN)
      
    def openNum(self):
      """ None -> number of open triangles"""
      x = 0       #Count of how many open triangles there are 
      for i in range(len(self.adjTris)): #Check all adjacent triangles. O(1), can only run max 3 times
        if self.adjTris[i].nextTri == None and self.adjTris[i].prevTri == None: 
          x += 1
      return x
    
    def leastFull(self):
      """ None -> least full triangle of adjacent triangles """
      least = [4,4] #Upper bound for replacment
      for i in range(len(self.adjTris)):  #Check all adjacent triangles. O(1), can only run max 3 times
        cur = self.adjTris[i] 
        if cur.nextTri == None and cur.prevTri == None:  #We only want to check elements of the adjTris that aren't part of a strip
          x = cur.openNum()
          if x < least[1]:    #Keep track of the least value'd adjacent triangle
            least = [i,x]
      return least[0]


# Draw an arrow between two points.

def drawArrow( x0,y0, x1,y1 ):

    d = math.sqrt( (x1-x0)*(x1-x0) + (y1-y0)*(y1-y0) )

    vx = (x1-x0) / d      # unit direction (x0,y0) -> (x1,y1)
    vy = (y1-y0) / d

    vpx = -vy             # unit direction perpendicular to (vx,vy)
    vpy = vx

    xa = x0 + 0.15*r*vx # arrow tail
    ya = y0 + 0.15*r*vy

    xb = x1 - 0.15*r*vx # arrow head
    yb = y1 - 0.15*r*vy

    xc = xb - 2*r*vx + 0.5*r*vpx # arrow outside left
    yc = yb - 2*r*vy + 0.5*r*vpy

    xd = xb - 2*r*vx - 0.5*r*vpx # arrow outside right
    yd = yb - 2*r*vy - 0.5*r*vpy

    glBegin( GL_LINES )
    glVertex2f( xa, ya )
    glVertex2f( 0.5*(xc+xd), 0.5*(yc+yd) )
    glEnd()

    glBegin( GL_LINE_LOOP )
    glVertex2f( xb, yb )
    glVertex2f( xc, yc )
    glVertex2f( xd, yd )
    glEnd()
      
      

# Determine whether three points make a left or right turn

LEFT_TURN  = 1
RIGHT_TURN = 2
COLLINEAR  = 3

def turn( a, b, c ):

    det = (a[0]-c[0]) * (b[1]-c[1]) - (b[0]-c[0]) * (a[1]-c[1])

    if det > 0:
        return LEFT_TURN
    elif det < 0:
        return RIGHT_TURN
    else:
        return COLLINEAR



# ================================================================
# ================================================================
# ================================================================

# Build a set of triangle strips that cover all of the given
# triangles.  The goal is to make the strips as long as possible
# (i.e. to have the fewest strip that cover all triangles).
#
# Follow the instructions in A2.txt.
#
# This function does not return anything.  The strips are formed by
# modifying the 'nextTri' and 'prevTri' pointers in each triangle.  
                                                    
def buildTristrips( triangles ):
    """ [List of triangles] -> Creation of triangles strips"""
    count = 0         #Number or strips
    i = 0
    while i!=len(triangles)-1:        #If we haven't checked all elements of triangles, we keep making strips,
      #Not sure if this is considered O(n), it should never run n times, but mby (1/k)n times?
      i = 0
      start = triangles[i]            #Just start at first triangle, a triangles adjacent triangles can change every iteration, so we just start at the beggining every time
      op2 = None                      #Second option variable

      while (start.openNum()!=1 or start.nextTri != None) and i<len(triangles)-1:       #O(n), will usually run n times, but won't if a corner is found
      #While  "not a corner"    or  "open triangle"       and  "in range of triangles"        
     
        if start.openNum() == 2 and op2 == None:      #If we happen to find a triangle thats an edge, I keep track of it
          op2 = start
##        elif start.openNum() == 0 and start.nextTri == None and start.prevTri == None: #Can only be done if prevTri = "Head". also makes it run much slower for some reason
##          op2 = start   
        i += 1
        start = triangles[i]    
          
      if start.openNum()!=1 and op2!=None:      #If we made it though the first loop and we didn't find a corner but we did find an edge, use the edge
          start = op2 
          i = 0       #just changing i so while loop dosen't end cause we can still have elements to check

      if i!=len(triangles)-1:
        count += 1                 #Mark that we are making a new list
        #start.prevTri = "Head"    #Can't do this, messes up pressing "f" to reverse arrows
        current = start            #idk, just for ease of reading
        while current.openNum() != 0:             #while we don't have a triangle with no open triangles (full). O(n), could theoretically run n times, but that would create a 1 perfect strip
          x = current.leastFull()                 #Find best adjacent triangle
          current.nextTri = current.adjTris[x]    #Step current
          current.adjTris[x].prevTri = current
          current = current.adjTris[x]
        #display(wait=True)
          
    for tri in triangles:           #O(n)
      if tri.openNum() == 0 and tri.nextTri == None and tri.prevTri==None:  #Checking if there are any stips of length 1
        count+=1
    ###I needed this because if I try and do it in the while loop, when ever it finds a triangle by itself
    ###The condition "while current.openNum()!=0" isn't meet. Thus the singlton triangle will never get any
    ###next or prev pointers, making the algorithm keep choosing as there is no way to identify it has been checked already
    ###creating an infinite loop. 

    #Overall, O(n^2) if the first while loop is O(n)
    print( 'Generated %d tristrips' % count )


# ================================================================
# ================================================================
# ================================================================



# Set up the display and draw the current image

windowLeft   = None
windowRight  = None
windowTop    = None
windowBottom = None

def display( wait=False ):

    global lastKey, windowLeft, windowRight, windowBottom, windowTop
    
    # Handle any events that have occurred

    glfw.poll_events()

    # Set up window

    glClearColor( 1,1,1,0 )
    glClear( GL_COLOR_BUFFER_BIT )
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL )

    glMatrixMode( GL_PROJECTION )
    glLoadIdentity()

    glMatrixMode( GL_MODELVIEW )
    glLoadIdentity()

    if maxX-minX > maxY-minY: # wider point spread in x direction
        windowLeft = -0.1*(maxX-minX)+minX
        windowRight = 1.1*(maxX-minX)+minX
        windowBottom = windowLeft
        windowTop    = windowRight
    else: # wider point spread in y direction
        windowTop    = -0.1*(maxY-minY)+minY
        windowBottom = 1.1*(maxY-minY)+minY
        windowLeft   = windowBottom
        windowRight  = windowTop

    glOrtho( windowLeft, windowRight, windowBottom, windowTop, 0, 1 )

    # Draw triangles

    for tri in allTriangles:
        tri.draw()

    # Draw pointers.  Do this *after* the triangles (above) so that the
    # triangle drawing doesn't overlay the pointers.

    for tri in allTriangles:
        tri.drawPointers()

    # Show window

    glfw.swap_buffers( window )

    # Maybe wait until the user presses 'p' to proceed
    
    if wait:

        #sys.stderr.write( 'Press "p" to proceed ' )
        sys.stderr.flush()

        lastKey = None
        while lastKey != 80: # wait for 'p'
            glfw.wait_events()
            display()

        #sys.stderr.write( '\r                     \r' )
        sys.stderr.flush()


    

# Handle keyboard input

def keyCallback( window, key, scancode, action, mods ):

    global lastKey, showForwardLinks
    
    if action == glfw.PRESS:
    
        if key == glfw.KEY_ESCAPE: # quit upon ESC
            sys.exit(0)
        elif key == ord('F'): # toggle forward/backward link display
            showForwardLinks = not showForwardLinks
        else:
            lastKey = key



# Handle window reshape


def windowReshapeCallback( window, newWidth, newHeight ):

    global windowWidth, windowHeight

    windowWidth  = newWidth
    windowHeight = newHeight



# Handle mouse click/release

def mouseButtonCallback( window, btn, action, keyModifiers ):

    if action == glfw.PRESS:

        # Find point under mouse

        x,y = glfw.get_cursor_pos( window ) # mouse position

        wx = (x-0)/float(windowWidth)  * (windowRight-windowLeft) + windowLeft
        wy = (windowHeight-y)/float(windowHeight) * (windowTop-windowBottom) + windowBottom

        selectedTri = None
        for tri in allTriangles:
            if tri.containsPoint( [wx, wy] ):
                selectedTri = tri
                break

        # print triangle, toggle its highlight1, and toggle the highlight2s of its adjacent triangles

        if selectedTri:
            selectedTri.highlight1 = not selectedTri.highlight1
            print( '%s with adjacent %s' % (selectedTri, repr(selectedTri.adjTris)) )
            for t in selectedTri.adjTris:
                t.highlight2 = not t.highlight2


# Read triangles from a file

def readTriangles( f ):

    global allVerts
    
    errorsFound = False
    
    lines = f.readlines()

    # Read the vertices
    
    numVerts = int( lines[0] )
    allVerts = [ [float(c) for c in line.split()] for line in lines[1:numVerts+1] ]

    # Check that the vertices are valid

    for l,v in enumerate(allVerts):
        if len(v) != 2:
            print( 'Line %d: vertex does not have two coordinates.' % (l+2) )
            errorsFound = True

    # Read the triangles

    numTris = int( lines[numVerts+1] )
    triVerts =  [ [int(v) for v in line.split()] for line in lines[numVerts+2:] ]

    # Check that the triangle vertices are valid

    for l,tvs in enumerate(triVerts):
        if len(tvs) != 3:
            print( 'Line %d: triangle does not have three vertices.' % (l+2+numVerts) )
            errorsFound = True
        else:
            for v in tvs:
                if v < 0 or v >= numVerts:
                    print( 'Line %d: Vertex index is not in range [0,%d].' % (l+2+numVerts,numVerts-1) )
                    errorsFound = True

    # Build triangles

    tris = []

    for tvs in triVerts:
        theseVerts = tvs
        if turn( allVerts[tvs[0]], allVerts[tvs[1]], allVerts[tvs[2]] ) != COLLINEAR:
          tris.append( Triangle( tvs ) ) # (don't include degenerate triangles)

    # For each triangle, find and record its adjacent triangles
    #
    # This would normally take O(n^2) time if done by brute force, so
    # we'll exploit Python's hashed dictionary keys.

    if False:

        for tri in tris: # brute force
            adjTris = []
            for i in range(3):
                v0 = tri.verts[i % 3]
                v1 = tri.verts[(i+1) % 3]
                for tri2 in tris:
                    for j in range(3):
                        if v1 == tri2.verts[j % 3] and v0 == tri2.verts[(j+1) % 3]:
                            adjTris.append( tri2 )
                    if len(adjTris) == 3:
                        break
            tri.adjTris = adjTris

    else: # hashing
      
        edges = {}

        for tri in tris:
            for i in range(3):
                v0 = tri.verts[i % 3]
                v1 = tri.verts[(i+1) % 3]
                key = '%f-%f' % (v0,v1)
                edges[key] = tri

        for tri in tris:
            adjTris = []
            for i in range(3):
                v1 = tri.verts[i % 3] # find a reversed edge of an adjacent triangle
                v0 = tri.verts[(i+1) % 3]
                key = '%f-%f' % (v0,v1)
                if key in edges:
                    adjTris.append( edges[key] )
                if len(adjTris) == 3:
                    break
            tri.adjTris = adjTris

    print( 'Read %d points and %d triangles' % (numVerts,numTris) )

    if errorsFound:
        return []
    else:
        return tris

        
    
# Initialize GLFW and run the main event loop

def main():

    global window, allTriangles, minX, maxX, minY, maxY, r
    
    # Check command-line args
    sys.argv.append("data/10000")
    if len(sys.argv) < 2:
        print( 'Usage: %s filename' % sys.argv[0] )
        sys.exit(1)

    args = sys.argv[1:]
    while len(args) > 1:
        # if args[0] == '-x':
        #     pass
        args = args[1:]

    # Set up window
  
    if not glfw.init():
        print( 'Error: GLFW failed to initialize' )
        sys.exit(1)

    window = glfw.create_window( windowWidth, windowHeight, "Assignment 2", None, None )

    if not window:
        glfw.terminate()
        print( 'Error: GLFW failed to create a window' )
        sys.exit(1)

    glfw.make_context_current( window )
    glfw.swap_interval( 1 )
    glfw.set_key_callback( window, keyCallback )
    glfw.set_window_size_callback( window, windowReshapeCallback )
    glfw.set_mouse_button_callback( window, mouseButtonCallback )

    # Read the triangles.  This also fills in the global 'allVerts'.

    with open( args[0], 'rb' ) as f:
        allTriangles = readTriangles( f )

    if allTriangles == []:
        return

    # Get bounding box of points

    minX = min( p[0] for p in allVerts )
    maxX = max( p[0] for p in allVerts )
    minY = min( p[1] for p in allVerts )
    maxY = max( p[1] for p in allVerts )

    # Adjust point radius in proportion to bounding box
    
    if maxX-minX > maxY-minY:
        r *= maxX-minX
    else:
        r *= maxY-minY

    # Run the code
    
    buildTristrips( allTriangles )

    # Show result and wait to exit

    display( wait=True )
    
    glfw.destroy_window( window )
    glfw.terminate()
    


if __name__ == '__main__':
    main()
