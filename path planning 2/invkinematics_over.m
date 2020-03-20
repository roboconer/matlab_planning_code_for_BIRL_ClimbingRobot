 function [a1,a2,a3,a4,a5]=invkinematics_over(r11,r12,r13,px,r21,r22,r23,py,r31,r32,r33,pz)
 L1=280;L2=106;L3=375; 
 if px<=0   
   a1=atan2(-py,-px);
   b234(1)=atan2(r13*cos(a1)+r23*sin(a1), r33);
   b234(2)=atan2(r13*cos(a1)+r23*sin(a1), r33)-2*pi;
   a5=atan2(-sin(a1)*r11+cos(a1)*r21,-sin(a1)*r12+cos(a1)*r22);
   a2=inf;a3=inf; a4=inf;                                                             %%%%%先给各关节一个初始值
    for i=1:2
        for j=1:2     
             v1=cos(a1)*px+sin(a1)*py-(L1+L2)*sin(b234(i));
             v2=pz+L1+L2-(L1+L2)*cos(b234(i));
             % cos(a3)=((v1)^2+(v2)^2-2*(L3)^2)/(2*(L3)^2);
             ca3=((v1)^2+(v2)^2-2*(L3)^2)/(2*(L3)^2);
             if abs(ca3)>1
                 break
             else
             b3(j)=atan2((-1)^j*(1-ca3^2)^0.5,ca3);
              if abs(b3(j))>pi/2
                 break
              end
             end
                v3=-(px*sin(b234(i))*cos(a1)+py*sin(b234(i))*sin(a1)+cos(b234(i))*(pz+L1+L2))+L1+L2;
                v4=-(px*cos(b234(i))*cos(a1)+py*cos(b234(i))*sin(a1)-(pz+L1+L2)*sin(b234(i)));
                % sin(a4)=((cos(a3)+1)*v3-sin(a3)*v4)/(2*L3*(cos(a3)+1));
                sa4(j)=((cos(b3(j))+1)*v3-sin(b3(j))*v4)/(2*L3*(cos(b3(j))+1));
                ca4(j)=((cos(b3(j))+1)*v4+sin(b3(j))*v3)/(2*L3*(cos(b3(j))+1));
                if abs(sa4)>1
                    break
                else
                b4(j)=atan2(sa4(j),ca4(j));
                b2=b234(i)-b3(j)-b4(j); 
                end
                if  abs(b3(j))<=pi/2+10^(-5)&&abs(b4(j)+pi/2)<=pi/2+10^(-5)&&abs(b2+pi/2)<=pi/2+10^(-5)&&abs(b2)<=a2
                    a3=b3(j);a2=b2;a4=b4(j);
%                 elseif abs(b3(j))<=pi/2&&abs(b4(s)+pi/2)<=pi/2&&abs(b2+pi/2)<=pi/2&&abs(b2)>=abs(a2)
%                        a3=a3;a2=a2;a4=a4;
                end
        end
    end
elseif px>0&&py>=0
        a1=atan2(-py,-px)+pi;
   b234(1)=atan2(r13*cos(a1)+r23*sin(a1), r33);
   b234(2)=atan2(r13*cos(a1)+r23*sin(a1), r33)-2*pi;
   a5=atan2(-sin(a1)*r11+cos(a1)*r21,-sin(a1)*r12+cos(a1)*r22);
   a2=inf;a3=inf; a4=inf;                                                             %%%%%先给各关节一个初始值
    for i=1:2
        for j=1:2     
             v1=cos(a1)*px+sin(a1)*py-(L1+L2)*sin(b234(i));
             v2=pz+L1+L2-(L1+L2)*cos(b234(i));
             % cos(a3)=((v1)^2+(v2)^2-2*(L3)^2)/(2*(L3)^2);
             ca3=((v1)^2+(v2)^2-2*(L3)^2)/(2*(L3)^2);
             if abs(ca3)>1
                 break
             else
             b3(j)=atan2((-1)^j*(1-ca3^2)^0.5,ca3);
              if abs(b3(j))>pi/2
                 break
              end
             end
                v3=-(px*sin(b234(i))*cos(a1)+py*sin(b234(i))*sin(a1)+cos(b234(i))*(pz+L1+L2))+L1+L2;
                v4=-(px*cos(b234(i))*cos(a1)+py*cos(b234(i))*sin(a1)-(pz+L1+L2)*sin(b234(i)));
                % sin(a4)=((cos(a3)+1)*v3-sin(a3)*v4)/(2*L3*(cos(a3)+1));
                sa4(j)=((cos(b3(j))+1)*v3-sin(b3(j))*v4)/(2*L3*(cos(b3(j))+1));
                ca4(j)=((cos(b3(j))+1)*v4+sin(b3(j))*v3)/(2*L3*(cos(b3(j))+1));
                if abs(sa4)>1
                    break
                else
                b4(j)=atan2(sa4(j),ca4(j));
                b2=b234(i)-b3(j)-b4(j); 
                end
                if  abs(b3(j))<=pi/2+10^(-5)&&abs(b4(j)+pi/2)<=pi/2+10^(-5)&&abs(b2+pi/2)<=pi/2+10^(-5)&&abs(b2)<=a2
                    a3=b3(j);a2=b2;a4=b4(j);
%                 elseif abs(b3(j))<=pi/2&&abs(b4(s)+pi/2)<=pi/2&&abs(b2+pi/2)<=pi/2&&abs(b2)>=abs(a2)
%                        a3=a3;a2=a2;a4=a4;
                end
        end
    end
else
    a1=atan2(-py,-px)-pi;
   b234(1)=atan2(r13*cos(a1)+r23*sin(a1), r33);
   b234(2)=atan2(r13*cos(a1)+r23*sin(a1), r33)-2*pi;
   a5=atan2(-sin(a1)*r11+cos(a1)*r21,-sin(a1)*r12+cos(a1)*r22);
   a2=inf;a3=inf; a4=inf;                                                             %%%%%先给各关节一个初始值
    for i=1:2
        for j=1:2     
             v1=cos(a1)*px+sin(a1)*py-(L1+L2)*sin(b234(i));
             v2=pz+L1+L2-(L1+L2)*cos(b234(i));
             % cos(a3)=((v1)^2+(v2)^2-2*(L3)^2)/(2*(L3)^2);
             ca3=((v1)^2+(v2)^2-2*(L3)^2)/(2*(L3)^2);
             if abs(ca3)>1
                 break
             else
             b3(j)=atan2((-1)^j*(1-ca3^2)^0.5,ca3);
              if abs(b3(j))>pi/2
                 break
              end
             end
                v3=-(px*sin(b234(i))*cos(a1)+py*sin(b234(i))*sin(a1)+cos(b234(i))*(pz+L1+L2))+L1+L2;
                v4=-(px*cos(b234(i))*cos(a1)+py*cos(b234(i))*sin(a1)-(pz+L1+L2)*sin(b234(i)));
                % sin(a4)=((cos(a3)+1)*v3-sin(a3)*v4)/(2*L3*(cos(a3)+1));
                sa4(j)=((cos(b3(j))+1)*v3-sin(b3(j))*v4)/(2*L3*(cos(b3(j))+1));
                ca4(j)=((cos(b3(j))+1)*v4+sin(b3(j))*v3)/(2*L3*(cos(b3(j))+1));
                if abs(sa4)>1
                    break
                else
                b4(j)=atan2(sa4(j),ca4(j));
                b2=b234(i)-b3(j)-b4(j); 
                end
                if  abs(b3(j))<=pi/2+10^(-5)&&abs(b4(j)+pi/2)<=pi/2+10^(-5)&&abs(b2+pi/2)<=pi/2+10^(-5)&&abs(b2)<=a2
                    a3=b3(j);a2=b2;a4=b4(j);
%                 elseif abs(b3(j))<=pi/2&&abs(b4(s)+pi/2)<=pi/2&&abs(b2+pi/2)<=pi/2&&abs(b2)>=abs(a2)
%                        a3=a3;a2=a2;a4=a4;
                end
        end
    end
end