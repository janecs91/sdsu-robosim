addpath('../robot');
bot = RollBot(terrain, path);


wheelVectorPos = obj.rotateVector([obj.wheelRadius 0], orientation);
wheelVectorNeg = obj.rotateVector([-obj.wheelRadius 0], orientation);
x0 = newState.endPositions(:,1);
y0 = newState.endPositions(:,2);
x1 = x0 - wheelVectorNeg;
x2 = x0 + wheelVectorPos;
y1 = y0 - wheelVectorNeg;
y2 = y0 + wheelVectorPos;