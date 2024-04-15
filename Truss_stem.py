import PyQt5.QtGui as qtg
import PyQt5.QtCore as qtc
import PyQt5.QtWidgets as qtw
import math

# used classes and code from wk11 repo from Dr.Smay

class Position():
    """
    I made this position for holding a position in 3D space (i.e., a point).  I've given it some ability to do
    vector arithmitic and vector algebra (i.e., a dot product).  I could have used a numpy array, but I wanted
    to create my own.  This class uses operator overloading as explained in the class.
    """

    def __init__(self, pos=None, x=None, y=None, z=None):
        """
        x, y, and z have the expected meanings
        :param pos: a tuple (x,y,z)
        :param x: float
        :param y: float
        :param z: float
        """
        # set default values
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        # unpack position from a tuple if given
        if pos is not None:
            self.x, self.y, self.z = pos
        # override the x,y,z defaults if they are given as arguments
        self.x = x if x is not None else self.x
        self.y = y if y is not None else self.y
        self.z = z if z is not None else self.z

    # region operator overloads $NEW$ 4/7/21
    def __eq__(self, other):
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        return True

    # this is overloading the addition operator.  Allows me to add Position objects with simple math: c=a+b, where
    # a, b, and c are all position objects.
    def __add__(self, other):
        return Position((self.x + other.x, self.y + other.y, self.z + other.z))

    # this overloads the iterative add operator
    def __iadd__(self, other):
        if other in (float, int):
            self.x += other
            self.y += other
            self.z += other
            return self
        if type(other) == Position:
            self.x += other.x
            self.y += other.y
            self.z += other.z
            return self

    # this is overloading the subtraction operator.  Allows me to subtract Positions. (i.e., c=b-a)
    def __sub__(self, other):
        return Position((self.x - other.x, self.y - other.y, self.z - other.z))

    # this overloads the iterative subtraction operator
    def __isub__(self, other):
        if other in (float, int):
            self.x -= other
            self.y -= other
            self.z -= other
            return self
        if type(other) == Position:
            self.x -= other.x
            self.y -= other.y
            self.z -= other.z
            return self

    # this is overloading the multiply operator.  Allows me to multiply a scalar or do a dot product (i.e., b=s*a or c=b*a)
    def __mul__(self, other):
        if type(other) in (float, int):
            return Position((self.x * other, self.y * other, self.z * other))
        if type(other) is Position:
            return Position((self.x * other.x, self.y * other.y, self.z * other.z))

    # this is overloading the __rmul__ operator so that s*Pt works.
    def __rmul__(self, other):
        return self * other

    # this is overloading the *= operator.  Same as a = Position((a.x*other, a.y*other, a.z*other))
    def __imul__(self, other):
        if type(other) in (float, int):
            self.x *= other
            self.y *= other
            self.z *= other
            return self

    # this is overloading the division operator.  Allows me to divide by a scalar (i.e., b=a/s)
    def __truediv__(self, other):
        if type(other) in (float, int):
            return Position((self.x / other, self.y / other, self.z / other))

    # this is overloading the /= operator.  Same as a = Position((a.x/other, a.y/other, a.z/other))
    def __idiv__(self, other):
        if type(other) in (float, int):
            self.x /= other
            self.y /= other
            self.z /= other
            return self

    # endregion

    def set(self, strXYZ=None, tupXYZ=None):
        # set position by string or tuple
        if strXYZ is not None:
            cells = strXYZ.replace('(', '').replace(')', '').strip().split(',')
            x, y, z = float(cells[0]), float(cells[1]), float(cells[2])
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)
        elif tupXYZ is not None:
            x, y, z = tupXYZ  # [0], strXYZ[1],strXYZ[2]
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)

    def getTup(self):  # return (x,y,z) as a tuple
        return (self.x, self.y, self.z)

    def getStr(self, nPlaces=3):
        return '{}, {}, {}'.format(round(self.x, nPlaces), round(self.y, nPlaces), round(self.z, nPlaces))

    def mag(self):  # normal way to calculate magnitude of a vector
        return (self.x ** 2 + self.y ** 2 + self.z ** 2) ** 0.5

    def normalize(self):  # typical way to normalize to a unit vector
        l = self.mag()
        if l <= 0.0:
            return
        self.__idiv__(l)

    def getAngleRad(self):
        """
        Gets angle of position relative to an origin (0,0) in the x-y plane
        :return: angle in x-y plane in radians
        """
        l = self.mag()
        if l <= 0.0:
            return 0
        if self.y >= 0.0:
            return math.acos(self.x / l)
        return 2.0 * math.pi - math.acos(self.x / l)

    def getAngleDeg(self):
        """
        Gets angle of position relative to an origin (0,0) in the x-y plane
        :return: angle in x-y plane in degrees
        """
        return 180.0 / math.pi * self.getAngleRad()


class Material():
    def __init__(self, uts=None, ys=None, modulus=None, staticFactor=None):
        self.uts = uts
        self.ys = ys
        self.E = modulus
        self.staticFactor = staticFactor


class Node():
    def __init__(self, name=None, position=None):
        self.name = name
        self.position = position if position is not None else Position()

    def __eq__(self, other):
        """
        This overloads the == operator such that I can compare two nodes to see if they are the same node.  This is
        useful when reading in nodes to make sure I don't get duplicate nodes
        """
        if self.name != other.name:
            return False
        if self.position != other.position:
            return False
        return True


class Link():
    def __init__(self, name="", node1="1", node2="2", length=None, angleRad=None):
        """
        Basic definition of a link contains a name and names of node1 and node2
        """
        self.name = ""
        self.node1_Name = node1
        self.node2_Name = node2
        self.length = None
        self.angleRad = None

    def __eq__(self, other):
        """
        This overloads the == operator for comparing equivalence of two links.
        """
        if self.node1_Name != other.node1_Name: return False
        if self.node2_Name != other.node2_Name: return False
        if self.length != other.length: return False
        if self.angleRad != other.angleRad: return False
        return True

    def set(self, node1=None, node2=None, length=None, angleRad=None):
        self.node1_Name = node1
        self.node2_Name = node2
        self.length = length
        self.angleRad = angleRad


class TrussModel():
    def __init__(self):
        self.title = None
        self.links = []
        self.nodes = []
        self.material = Material()

    def getNode(self, name):
        for n in self.nodes:
            if n.name == name:
                return n


class TrussController():
    def __init__(self):
        self.truss = TrussModel()
        self.view = TrussView()

    def ImportFromFile(self, data):
        """
        Data is the list of strings read from the data file.
        We need to parse this file and build the lists of nodes and links that make up the truss.
        Also, we need to parse the lines that give the truss title, material (and strength values).

        Reading Nodes:
        I create a new node object and the set its name and position.x and position.y values.  Next, I check to see
        if the list of nodes in the truss model has this node with self.hasNode(n.name).  If the trussModel does not
        contain the node, I append it to the list of nodes

        Reading Links:
        The links should come after the nodes.  Each link has a name and two node names.  See method addLink
        """
        for line in data:
            if len(line) > 1:
                cells = line.strip().split(',')
                if cells[0] == '#':
                    pass
                else:  # need to parse the line
                    if cells[0].lower().strip() == 'title':
                        self.truss.title = cells[1].replace("'", "").strip()
                    elif cells[0].lower().strip() == 'material':
                        self.truss.material.uts = float(cells[1].replace("'", "").strip())
                        self.truss.material.ys = float(cells[2].replace("'", "").strip())
                        self.truss.material.E = float(cells[1].replace("'", "").strip())
                    elif cells[0].lower().strip() == 'static_factor':
                        self.truss.material.staticFactor = float(cells[1].replace("'", "").strip())
                    elif cells[0].lower().strip() == 'node':
                        n = Node()
                        n.name = cells[1].replace("'", "").strip()
                        n.position.x = float(cells[2].replace("'", "").strip())
                        n.position.y = float(cells[3].replace("'", "").strip())
                        if not self.hasNode(n.name):
                            self.addNode(n)
                    elif cells[0].lower().strip() == 'link':
                        l = Link()
                        l.name = cells[1].replace("'", "").strip()
                        l.node1_Name = cells[2].replace("'", "").strip()
                        l.node2_Name = cells[3].replace("'", "").strip()
                        self.addLink(l)
        self.calcLinkVals()
        self.displayReport()
        self.drawTruss()

    def hasNode(self, name):
        for n in self.truss.nodes:
            if n.name == name:
                return True
        return False

    def addNode(self, node):
        self.truss.nodes.append(node)

    def getNode(self, name):
        for n in self.truss.nodes:
            if n.name == name:
                return n

    def addLink(self, link):
        self.truss.links.append(link)

    def calcLinkVals(self):
        for l in self.truss.links:
            n1 = None
            n2 = None
            if self.hasNode(l.node1_Name):
                n1 = self.getNode(l.node1_Name)
            if self.hasNode(l.node2_Name):
                n2 = self.getNode(l.node2_Name)
            if n1 is not None and n2 is not None:
                r = n2.position - n1.position
                l.length = r.mag()
                l.angleRad = r.getAngleRad()

    def setDisplayWidgets(self, args):
        self.view.setDisplayWidgets(args)

    def displayReport(self):
        self.view.displayReport(truss=self.truss)

    def drawTruss(self):
        self.view.buildScene(truss=self.truss)


class TrussView():
    def __init__(self):
        # setup widgets for display.  redefine these when you have a gui to work with using setDisplayWidgets
        self.scene = qtw.QGraphicsScene()
        self.le_LongLinkName = qtw.QLineEdit()
        self.le_LongLinkNode1 = qtw.QLineEdit()
        self.le_LongLinkNode2 = qtw.QLineEdit()
        self.le_LongLinkLength = qtw.QLineEdit()
        self.te_Report = qtw.QTextEdit()
        self.gv = qtw.QGraphicsView()

        # region setup pens and brushes and scene
        # make the pens first
        # a thick darkGray pen
        self.penLink = qtg.QPen(qtc.Qt.darkGray)
        self.penLink.setWidth(4)
        # a medium darkBlue pen
        self.penNode = qtg.QPen(qtc.Qt.darkBlue)
        self.penNode.setStyle(qtc.Qt.SolidLine)
        self.penNode.setWidth(1)
        # a pen for the grid lines
        self.penGridLines = qtg.QPen()
        self.penGridLines.setWidth(1)
        # I wanted to make the grid lines more subtle, so set alpha=25
        self.penGridLines.setColor(qtg.QColor.fromHsv(197, 144, 228, alpha=50))
        # now make some brushes
        # build a brush for filling with solid red
        self.brushFill = qtg.QBrush(qtc.Qt.darkRed)
        # a brush that makes a hatch pattern
        self.brushNode = qtg.QBrush(qtg.QColor.fromCmyk(0, 0, 255, 0, alpha=100))
        # a brush for the background of my grid
        self.brushGrid = qtg.QBrush(qtg.QColor.fromHsv(87, 98, 245, alpha=128))
        # endregion

    def setDisplayWidgets(self, args):
        self.te_Report = args[0]
        self.le_LongLinkName = args[1]
        self.le_LongLinkNode1 = args[2]
        self.le_LongLinkNode2 = args[3]
        self.le_LongLinkLength = args[4]
        self.gv = args[5]
        self.gv.setScene(self.scene)

    def displayReport(self, truss=None):
        st = '\tTruss Design Report\n'
        st += 'Title:  {}\n'.format(truss.title)
        st += 'Static Factor of Safety:  {:0.2f}\n'.format(truss.material.staticFactor)
        st += 'Ultimate Strength:  {:0.2f}\n'.format(truss.material.uts)
        st += 'Yield Strength:  {:0.2f}\n'.format(truss.material.ys)
        st += 'Modulus of Elasticity:  {:0.2f}\n'.format(truss.material.E)
        st += '_____________Link Summary________________\n'
        st += 'Link\t(1)\t(2)\tLength\tAngle\n'
        longest = None
        for l in truss.links:
            if longest is None or l.length > longest.length:
                longest = l
            st += '{}\t{}\t{}\t{:0.2f}\t{:0.2f}\n'.format(l.name, l.node1_Name, l.node2_Name, l.length, l.angleRad)
        self.te_Report.setText(st)
        self.le_LongLinkName.setText(longest.name)
        self.le_LongLinkLength.setText("{:0.2f}".format(longest.length))
        self.le_LongLinkNode1.setText(longest.node1_Name)
        self.le_LongLinkNode2.setText(longest.node2_Name)

    def buildScene(self, truss=None):
        rect = qtc.QRect()
        rect.setTop(int(truss.nodes[0].position.y))  # floating values causing problems
        rect.setLeft(int(truss.nodes[0].position.x))  # floating values causing problems
        rect.setHeight(0)
        rect.setWidth(0)
        for n in truss.nodes:
            if n.position.y + 30 > rect.top():  # giving cushion to all sides so it doesnt look weird
                rect.setTop(int(n.position.y) + 30)
            if n.position.y - 30 < rect.bottom():
                rect.setBottom(int(n.position.y) - 30)
            if n.position.x + 30 > rect.right():
                rect.setRight(int(n.position.x) + 30)
            if n.position.x - 30 < rect.left():
                rect.setLeft(int(n.position.x) - 30)
        # rect.adjust(-50, 50, 50, -50) # no longer needed cause of cushion

        # clear out the old scene first
        self.scene.clear()

        # draw a grid
        self.drawAGrid(DeltaX=10, DeltaY=10, Height=abs(rect.height()), Width=abs(rect.width()),
                       CenterX=rect.center().x(), CenterY=rect.center().y())
        # draw the pipe network
        self.drawLinks(truss=truss)
        self.drawNodes(truss=truss)

    def drawAGrid(self, DeltaX=10, DeltaY=10, Height=320, Width=180, CenterX=120, CenterY=60):
        """
        This makes a grid for reference.  No snapping to grid enabled.
        :param DeltaX: grid spacing in x direction
        :param DeltaY: grid spacing in y direction
        :param Height: height of grid (y)
        :param Width: width of grid (x)
        :param CenterX: center of grid (x, in scene coords)
        :param CenterY: center of grid (y, in scene coords)
        :param Pen: pen for grid lines
        :param Brush: brush for background
        :return: nothing
        """
        Pen = self.penGridLines
        Brush = self.brushGrid
        height = self.scene.sceneRect().height() if Height is None else Height
        width = self.scene.sceneRect().width() if Width is None else Width
        left = self.scene.sceneRect().left() if CenterX is None else (CenterX - width / 2.0)
        right = self.scene.sceneRect().right() if CenterX is None else (CenterX + width / 2.0)
        top = -1.0 * self.scene.sceneRect().top() if CenterY is None else (CenterY + height / 2.0)
        bottom = -1.0 * self.scene.sceneRect().bottom() if CenterY is None else (CenterY - height / 2.0)
        Dx = DeltaX
        Dy = DeltaY
        pen = qtg.QPen() if Pen is None else Pen

        # make the background rectangle first
        if Brush is not None:
            rect = qtw.QGraphicsRectItem(left, -top, width, height)
            rect.setBrush(Brush)
            rect.setPen(pen)
            self.scene.addItem(rect)
        # draw the vertical grid lines
        x = left
        while x <= right:
            lVert = qtw.QGraphicsLineItem(x, -top, x, -bottom)
            lVert.setPen(pen)
            self.scene.addItem(lVert)
            x += Dx
        # draw the horizontal grid lines
        y = bottom
        while y <= top:
            lHor = qtw.QGraphicsLineItem(left, -y, right, -y)  # now flip y
            lHor.setPen(pen)
            self.scene.addItem(lHor)
            y += Dy

    def drawLinks(self, truss=None):
        scene = self.scene
        penLink = self.penLink
        for l in truss.links:
            n1 = truss.getNode(l.node1_Name)
            n2 = truss.getNode(l.node2_Name)
            x1 = n1.position.x
            y1 = n1.position.y
            x2 = n2.position.x
            y2 = n2.position.y
            st = 'link: ' + l.name + '\n'
            st += 'length = {:0.2f}\n'.format(l.length)
            st += 'angle deg = {:0.2f}'.format(l.angleRad * 180.0 / math.pi)

            # call rigid link and use the node positions of the links to set coordinates
            # set colors to my favorite color cause i could barely see the double orange
            riglink = RigidLink(x1, -y1, x2, -y2, pen=qtg.QPen(qtg.QColor("blue")),
                                brush=qtg.QBrush(qtg.QColor.fromHsv(315, 255, 255, 100)))
            riglink.setToolTip(st)
            # line.setPen(self.penLink) # i dont think this is needed anymore
            # add rigid link to scene
            scene.addItem(riglink)

    def drawNodes(self, truss=None, scene=None):
        if scene is None:
            scene = self.scene
        penNode = self.penNode
        brushNode = self.brushNode
        penNodeOutline = qtg.QPen() if penNode is None else penNode
        penNodeLabel = qtg.QPen(qtg.QColor("magenta"))
        for n in truss.nodes:
            x = n.position.x
            y = n.position.y

            if n.name.lower() == 'left':  # if node is the anchored node put rigid pivot on it
                scene.addItem(RigidPivotPoint(x, y, pivotHeight=10, pivotWidth=20,
                                              pen=qtg.QPen(qtg.QColor("black"))))
            elif n.name.lower() == 'right':  # if node is the anchored node put rigid pivot on it
                scene.addItem(RigidPivotPoint(x, y, pivotHeight=10, pivotWidth=20,
                                              pen=qtg.QPen(qtg.QColor("black"))))
            else:
                self.drawALabel(x - 15, y + 15, str=n.name, pen=penNodeLabel)

    def drawALabel(self, x, y, str='', pen=None, brush=None, tip=None):
        """
        I've decided that x,y are the center of the label.  Find corner based on label width and height.
        """
        scene = self.scene
        lbl = qtw.QGraphicsTextItem(str)
        w = lbl.boundingRect().width()
        h = lbl.boundingRect().height()
        lbl.setX(x - w / 2.0)
        lbl.setY(-y - h / 2.0)
        if tip is not None:
            lbl.setToolTip(tip)
        if pen is not None:
            lbl.setDefaultTextColor(pen.color())
        if brush is not None:
            # this makes a nice background
            bkg = qtw.QGraphicsRectItem(lbl.x(), lbl.y(), w, h)
            bkg.setBrush(brush)
            outlinePen = qtg.QPen(brush.color())
            bkg.setPen(outlinePen)
            scene.addItem(bkg)
        scene.addItem(lbl)

    def drawACircle(self, centerX, centerY, Radius, angle=0, brush=None, pen=None, name=None, tooltip=None):
        scene = self.scene
        # ellipse = qtw.QGraphicsEllipseItem(centerX - Radius, centerY - Radius, 2 * Radius, 2 * Radius)
        ellipse = qtw.QGraphicsEllipseItem(centerX - Radius, -1.0 * (centerY + Radius), 2 * Radius,
                                           2 * Radius)  # $NEW$ 4/7/21 flip y
        if pen is not None:
            ellipse.setPen(pen)
        if brush is not None:
            ellipse.setBrush(brush)
        if name is not None:
            ellipse.setData(0, name)
        if tooltip is not None:
            ellipse.setToolTip(tooltip)
        scene.addItem(ellipse)


# yoinking the rigid classes so that it doesnt rely on a seperate py file, pretty much
# copied and pasted with minor adjustments

class RigidLink(qtw.QGraphicsItem):
    def __init__(self, stX, stY, enX, enY, radius=10, parent=None, pen=None, brush=None):
        """
        This is a custom class for drawing a rigid link.  The paint function executes everytime the scene
        which holds the link is updated.  The steps to making the link are:
        1. Specify the start and end x,y coordinates of the link
        2. Specify the radius (i.e., the width of the link)
        3. Compute the length of the link
        3. Compute the angle of the link relative to the x-axis
        4. Compute the angle normal the angle of the length by adding pi/2
        5. Compute the rectangle that will contain the link (i.e., its bounding box)
        These steps are executed each time the paint function is invoked
        :param stX:
        :param stY:
        :param enX:
        :param enY:
        :param radius:
        :param parent:
        :param pen:
        :param brush:
        """
        super().__init__(parent)
        # step 1
        self.startX = stX
        self.startY = stY
        self.endX = enX
        self.endY = enY
        # step 2
        self.radius = radius
        # step 3
        self.length = self.linkLength()
        # step 4
        self.angle = self.linkAngle()
        # step 5
        self.normAngle = self.angle + math.pi / 2
        # step 6
        self.width = self.endX - self.startX + 2 * self.radius
        self.height = self.endY - self.startY + 2 * self.radius
        self.rect = qtc.QRectF(self.startX, self.startY, self.width, self.height)

        self.pen = pen
        self.brush = brush

    def boundingRect(self):
        return self.rect

    def linkLength(self):
        self.length = math.sqrt(math.pow(self.startX - self.endX, 2) + math.pow(self.startY - self.endY, 2))
        return self.length

    def linkAngle(self):
        self.angle = math.acos((self.endX - self.startX) / self.linkLength())
        self.angle *= -1 if (self.endY > self.startY) else 1
        return self.angle

    def paint(self, painter, option, widget=None):
        """
        This function creates a path painter the paints a semicircle around the start point (ccw), a straight line
        offset from the main axis of the link, a semicircle around the end point (ccw), and a straight line offset from
        the main axis.  It then assigns a pen and brush.  Finally, it draws a circle at the start and end points to
        indicate the pivot points.
        :param painter:
        :param option:
        :param widget:
        :return:
        """
        path = qtg.QPainterPath()
        len = self.linkLength()
        angLink = self.linkAngle() * 180 / math.pi
        perpAng = angLink + 90
        xOffset = self.radius * math.cos(perpAng * math.pi / 180)
        yOffset = -self.radius * math.sin(perpAng * math.pi / 180)
        rectStart = qtc.QRectF(self.startX - self.radius, self.startY - self.radius, 2 * self.radius, 2 * self.radius)
        rectEnd = qtc.QRectF(self.endX - self.radius, self.endY - self.radius, 2 * self.radius, 2 * self.radius)
        centerLinePen = qtg.QPen()
        centerLinePen.setStyle(qtc.Qt.DashDotLine)
        r, g, b, a = self.pen.color().getRgb()
        centerLinePen.setColor(qtg.QColor(r, g, b, 128))
        centerLinePen.setWidth(1)
        p1 = qtc.QPointF(self.startX, self.startY)
        p2 = qtc.QPointF(self.endX, self.endY)
        painter.setPen(centerLinePen)
        painter.drawLine(p1, p2)
        path.arcMoveTo(rectStart, perpAng)
        path.arcTo(rectStart, perpAng, 180)
        path.lineTo(self.endX - xOffset, self.endY - yOffset)
        path.arcMoveTo(rectEnd, perpAng + 180)
        path.arcTo(rectEnd, perpAng + 180, 180)
        path.lineTo(self.startX + xOffset, self.startY + yOffset)
        if self.pen is not None:
            painter.setPen(self.pen)  # Red color pen
        if self.brush is not None:
            painter.setBrush(self.brush)
        painter.drawPath(path)
        pivotStart = qtc.QRectF(self.startX - self.radius / 6, self.startY - self.radius / 6, self.radius / 3,
                                self.radius / 3)
        pivotEnd = qtc.QRectF(self.endX - self.radius / 6, self.endY - self.radius / 6, self.radius / 3,
                              self.radius / 3)
        painter.drawEllipse(pivotStart)
        painter.drawEllipse(pivotEnd)


class RigidPivotPoint(qtw.QGraphicsItem):
    def __init__(self, ptX, ptY, pivotHeight, pivotWidth, parent=None, pen=None, brush=None, rotation=0):
        super().__init__(parent)
        self.x = ptX
        self.y = ptY
        self.pen = pen
        self.brush = brush
        self.height = pivotHeight
        self.width = pivotWidth
        self.radius = min(self.height, self.width) / 4
        self.rect = qtc.QRectF(self.x - self.width / 2, self.y - self.radius, self.width, self.height + self.radius)
        self.rotationAngle = rotation

    def boundingRect(self):
        return self.rect

    def rotate(self, angle):
        self.rotationAngle = angle

    def paint(self, painter, option, widget=None):
        path = qtg.QPainterPath()
        radius = min(self.height, self.width) / 2
        rect = qtc.QRectF(self.x - self.width / 2, self.y - radius, self.width, self.height + radius)
        H = math.sqrt(math.pow(self.width / 2, 2) + math.pow(self.height, 2))
        phi = math.asin(radius / H)
        theta = math.asin(self.height / H)
        ang = math.pi - phi - theta
        l = H * math.cos(phi)
        x1 = self.x + self.width / 2
        y1 = self.y + self.height
        path.moveTo(x1, y1)
        x2 = l * math.cos(ang)
        y2 = l * math.sin(ang)
        path.lineTo(x1 + x2, y1 - y2)
        pivotRect = qtc.QRectF(self.x - radius, self.y - radius, 2 * radius, 2 * radius)
        stAng = math.pi / 2 - phi - theta
        spanAng = math.pi - 2 * stAng
        path.arcTo(pivotRect, stAng * 180 / math.pi, spanAng * 180 / math.pi)
        x4 = self.x - self.width / 2
        y4 = self.y + self.height
        path.lineTo(x4, y4)
        # path.arcTo(pivotRect,ang*180/math.pi, 90)
        if self.pen is not None:
            painter.setPen(self.pen)  # Red color pen
        if self.brush is not None:
            painter.setBrush(self.brush)
        painter.drawPath(path)

        pivotPtRect = qtc.QRectF(self.x - radius / 4, self.y - radius / 4, radius / 2, radius / 2)
        painter.drawEllipse(pivotPtRect)
        x5 = self.x - self.width
        x6 = self.x + self.width
        painter.drawLine(qtc.QPointF(x5, y4), qtc.QPointF(x6, y4))
        penOutline = qtg.QPen(qtc.Qt.NoPen)
        hatchbrush = qtg.QBrush(qtc.Qt.BDiagPattern)
        painter.setPen(penOutline)
        painter.setBrush(hatchbrush)
        support = qtc.QRectF(x5, y4, self.width * 2, self.height)
        painter.drawRect(support)
        self.setRotation(self.rotationAngle)
