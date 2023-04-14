function NextState = NextState(currentState, jointVel, dt, maxJointVel)

% set velocity range
for i = 1:6
    if jointVel(i) > maxJointVel(i)
        jointVel(i) = maxJointVel(i);
    end
end
tmp = currentState + jointVel*dt;

% set joint limits
tmp(1) = max(-pi,min(pi,tmp(1)));
tmp(2) = max(-pi,min(0,tmp(2))); 
tmp(3) = max(0,min(pi,tmp(3))); 
tmp(4) = max(-pi,min(pi,tmp(4))); 
tmp(5) = max(-pi,min(pi,tmp(5)));
tmp(6) = max(-pi,min(pi,tmp(6)));
NextState = tmp;

end