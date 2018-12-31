function pose = t2v(T)
  pose(1) = T(1,3);
  pose(2) = T(2,3);
  pose(3) = atan2(T(2,1)/T(1,1));
endfunction
