for i=1:21
AA(:,:,i)=[i;(i-1);Theta1(i); (i-1);Theta2(i); (i-1);Theta3(i); (i-1);Theta4(i); (i-1);Theta5(i);(i-1); vel1(i);(i-1); vel2(i); (i-1);vel3(i); (i-1);vel4(i); (i-1);vel5(i);(i-1); acc1(i); (i-1);acc2(i);(i-1); acc3(i); (i-1);acc4(i);(i-1); acc5(i)];
fid = fopen('pos.cpp','a');
fprintf(fid,'//%.0f\ng_P[%.0f].P[0] = %.3f;\ng_P[%.0f].P[1] =%.3f;\ng_P[%.0f].P[2] =%.3f;\ng_P[%.0f].P[3] =%.3f;\ng_P[%.0f].P[4] =%.3f;\ng_P[%.0f].V[0] = %.3f;\ng_P[%.0f].V[1] =%.3f;\ng_P[%.0f].V[2] =%.3f;\ng_P[%.0f].V[3] =%.3f;\ng_P[%.0f].V[4] =%.3f;\ng_P[%.0f].A[0] = %.3f;\ng_P[%.0f].A[1] =%.3f;\ng_P[%.0f].A[2] =%.3f;\ng_P[%.0f].A[3] =%.3f;\ng_P[%.0f].A[4] =%.3f;\n', AA(:,:,i));
fclose(fid);
end