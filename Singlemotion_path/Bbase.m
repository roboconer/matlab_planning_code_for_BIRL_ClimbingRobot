function result = Bbase(i,k,u,t)
%��i��k��B������,Deboor���Ƶݹ��㷨
%tΪ����,u(i)<=t<u(i+1),k=0ʱresult=1;
if(k==0)
    if(u(i)<=t && t<u(i+1))%ע��1=u(i)<=t<u(i+1)=1ʱ�����,����Ҫ��t<=u(i+1);
        result=1;
        return;
    else
        result=0;
        return;
    end
else
    if(u(i+k)-u(i)==0)
        alpha=0;
    else
        alpha=(t-u(i))/(u(i+k)-u(i));
    end
    if(u(i+k+1)-u(i+1)==0)
         beta=0;
    else
        beta=(u(i+k+1)-t)/(u(i+k+1)-u(i+1));
    end
end
result=alpha*Bbase(i,k-1,u,t)+beta*Bbase(i+1,k-1,u,t);
