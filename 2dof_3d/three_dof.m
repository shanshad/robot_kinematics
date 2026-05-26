clear all;
clc;

syms l1 l2 t1 t2 phi
l1_val=1;
l2_val=1;
t1_val=deg2rad(10);
t2_val=deg2rad(45);
phi_val=deg2rad(5);
x=(l1*cos(t1)+l2*cos(t1+t2))*cos(phi);
y = (l1*cos(t1)+l2*cos(t1+t2))*sin(phi);
z=l1*sin(t1)+l2*sin(t1+t2);
x_num=double(subs(x,[l1,l2,t1,t2,phi],[l1_val,l2_val,t1_val,t2_val,phi_val]));
y_num=double(subs(y,[l1,l2,t1,t2,phi],[l1_val,l2_val,t1_val,t2_val,phi_val]));
z_num=double(subs(z,[l1,l2,t1,t2,phi],[l1_val,l2_val,t1_val,t2_val,phi_val]));
j=[diff(x,t1) diff(x,t2) diff(x,phi);
    diff(y,t1) diff(y,t2) diff(y,phi);
    diff(z,t1) diff(z,t2) diff(z,phi)];
tol=0.001;
lr=0.1;
pin=[x_num;y_num;z_num];
q=[t1_val;t2_val;phi_val];
pdes=[0;0;2];
figure(1); clf;
ax = axes('XLim',[-2.2 2.2],'YLim',[-2.2 2.2],'ZLim',[-2.2 2.2]);
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('2-Link Robot Arm — IK Animation');
view(45, 25);

quiver3(0,0,0, 2,0,0, 'r','LineWidth',1.2,'MaxHeadSize',0.3);
quiver3(0,0,0, 0,2,0, 'g','LineWidth',1.2,'MaxHeadSize',0.3);
quiver3(0,0,0, 0,0,2, 'b','LineWidth',1.2,'MaxHeadSize',0.3);
text(2.1,0,0,'X','Color','r');
text(0,2.1,0,'Y','Color','g');
text(0,0,2.1,'Z','Color','b');

plot3(pdes(1),pdes(2),pdes(3),'rp','MarkerSize',18,'MarkerFaceColor','r');
hLink1  = plot3([0 0],[0 0],[0 0], 'b-', 'LineWidth', 6);
hLink2  = plot3([0 0],[0 0],[0 0], 'c-', 'LineWidth', 5);
hJoint0 = plot3(0, 0, 0, 'ko', 'MarkerSize',12,'MarkerFaceColor','k');
hJoint1 = plot3(0, 0, 0, 'ko', 'MarkerSize',10,'MarkerFaceColor',[0.4 0.4 0.4]);
hEE     = plot3(0, 0, 0, 'go', 'MarkerSize',12,'MarkerFaceColor','g');
hTrail  = plot3(nan, nan, nan, '-', 'LineWidth', 1.2, 'Color', [0 0.8 0.3 0.5]);
hText   = text(-2.1,-2.1, 2.1,'','FontSize',9,'FontName','Courier','VerticalAlignment','top');

x0 = 0; y0 = 0; z0 = 0;
trail_x = []; trail_y = []; trail_z = [];
for i=1:200
    x1 = l1_val * cos(t1_val) * cos(phi_val);
    y1 = l1_val * cos(t1_val) * sin(phi_val);
    z1 = l1_val * sin(t1_val);
    dx=pdes-pin;
    err = norm(dx);
    j_val=double(subs(j,[l1,l2,t1,t2,phi],[l1_val,l2_val,t1_val,t2_val,phi_val]));
    jinv=inv(j_val);
    dq=jinv*dx;
    q=q+lr*dq;
    t1_val=q(1);
    t2_val=q(2);
    phi_val=q(3);
    x2=double(subs(x,[l1,l2,t1,t2,phi],[l1_val,l2_val,t1_val,t2_val,phi_val]));
    y2=double(subs(y,[l1,l2,t1,t2,phi],[l1_val,l2_val,t1_val,t2_val,phi_val]));
    z2=double(subs(z,[l1,l2,t1,t2,phi],[l1_val,l2_val,t1_val,t2_val,phi_val]));
    pin=[x2;y2;z2];
    set(hLink1,  'XData',[x0 x1], 'YData',[y0 y1], 'ZData',[z0 z1]);
    set(hLink2,  'XData',[x1 x2], 'YData',[y1 y2], 'ZData',[z1 z2]);
    set(hJoint0, 'XData', x0, 'YData', y0, 'ZData', z0);
    set(hJoint1, 'XData', x1, 'YData', y1, 'ZData', z1);
    set(hEE,     'XData', x2, 'YData', y2, 'ZData', z2);

    trail_x(end+1) = x2;
    trail_y(end+1) = y2;
    trail_z(end+1) = z2;
    set(hTrail, 'XData', trail_x, 'YData', trail_y, 'ZData', trail_z);

    set(hText, 'String', sprintf(...
        'Iter : %d\nError: %.4f\nT1   : %.1f deg\nT2   : %.1f deg\nPhi  : %.1f deg',...
        i, err, rad2deg(t1_val), rad2deg(t2_val), rad2deg(phi_val)));

    drawnow;
    pause(0.03);
    if norm(dx)<tol
        display(i);
        break;
    end
end
display(pin);
    