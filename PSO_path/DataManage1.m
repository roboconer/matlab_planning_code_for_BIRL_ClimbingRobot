function [ ] = DataManage1( particle,velocity,pbest,gbest,max_iteration,swarm_size,particle_size,fitness_size )
%DATAMANAGE Summary of this function goes here
%   Detailed explanation goes here
 
temp=zeros(1,fitness_size);
for k=1:max_iteration
    temp(1,:)=gbest(1,:,k);
    save('ResultHistoryGbest','temp','-ascii','-tabs','-append');
end
temp(1,:)=gbest(1,:,max_iteration);
save('ResultFinalGbest','temp','-ascii','-tabs','-append');
end