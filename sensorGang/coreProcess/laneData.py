"""Get lane data."""


def getDataFromLines(self):
    """Get offset from lines."""
    self.currentSpeed = self.turningSpeed

    # Histogrammet har hittat båda linjerna
    if self.leftHistogram is not None and self.rightHistogram is not None:
        # Båda linjernas lutning har hittats
        if self.slopeLeft and self.slopeRight:
            # print("Case 1")
            self.newOffset = (0.6 * self.midpointHistogram +
                              0.5 * self.lineCenter)
            self.currentSpeed = self.normalSpeed

        # Endast vänstra linjens lutning har hittats
        elif self.slopeLeft:
            # print("Case 2")
            self.newOffset = (self.midpointHistogram -
                              1/self.slopeLeft * 420)

        # Endast högra linjens lutning har hittats
        elif self.slopeRight:
            # print("Case 3")
            self.newOffset = (self.midpointHistogram -
                              1/self.slopeRight * 370)

        # Inga lutningar har hittats
        else:
            # print("Case 4")
            self.newOffset = self.midpointHistogram

    # Histogrammet har endast hittat vänstra linjen
    elif self.leftHistogram is not None:
        # Vänstra linjens lutning har hittats
        if self.slopeLeft:
            # print("Case 5")
            midpointHistogram = (
                self.width - self.leftHistogram)/2 + self.center
            self.newOffset = midpointHistogram

            pass
        # Ingen lutning har hittats
        else:
            # print("Case 6")
            midpointHistogram = (
                self.width - self.leftHistogram)/2 + self.center
            self.newOffset = midpointHistogram

            pass

        pass

    # Histogrammet har endast hittat högra linjen
    elif self.rightHistogram is not None:
        # Vänstra linjens lutning har hittats
        if self.slopeRight:
            # print("Case 7")
            midpointHistogram = self.center - (self.rightHistogram)/2
            self.newOffset = midpointHistogram

            pass
        # Ingen lutning har hittats
        else:
            # print("Case 8")
            midpointHistogram = self.center - (self.rightHistogram)/2
            self.newOffset = midpointHistogram

            pass

    else:
        # print("Case 9")
        self.newOffset = self.lastOffset

        return

    self.newOffset -= (self.center + 40)

    self.newOffset = int(self.newOffset)


def getOffsetStraightLeft(self):
    """Get offset on straight, left line avalible."""
    self.currentSpeed = self.normalSpeed
    if self.leftHistogram is not None:
        self.newOffset = (self.leftHistogram - 140)*2
    else:
        self.newOffset = - 50


def getOffsetStraightRight(self):
    """Get offset on straight, right line avalible."""
    self.currentSpeed = self.normalSpeed
    if self.rightHistogram is not None:
        self.newOffset = (self.rightHistogram - 530)*2
    else:
        self.newOffset = 50


def getOffsetLeftTurn(self):
    """Get offset on left turn."""
    self.currentSpeed = self.turningSpeed - 10
    if self.leftHistogram is not None:
        self.newOffset = (self.leftHistogram - 140)*3.5
    else:
        self.newOffset = - 150


def getOffsetRightTurn(self):
    """Get offset on right turn."""
    self.currentSpeed = self.turningSpeed - 10
    if self.rightHistogram is not None:
        self.newOffset = (self.rightHistogram - 530)*3
    else:
        self.newOffset = 150

