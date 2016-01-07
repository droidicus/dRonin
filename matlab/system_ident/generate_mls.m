baseVal = 2;
powerVal = 9;

mlsSeq = mseq(baseVal, powerVal);

fid = fopen('mlsSeq.c','w');

N=numel(mlsSeq);
fprintf(fid, '#define MLS_LENGTH %d\n', N);
fprintf(fid, 'const sint8_t mlsSeq[MLS_LENGTH] =\n{');

for idx = 1:N
  fprintf(fid, '%d', mlsSeq(idx));
  
  if idx ~= N
    fprintf(fid, ',');
  end
  if ~mod((idx), 25)
    fprintf(fid, '\n');
  else
    fprintf(fid, ' ');
  end
end

fprintf(fid, '};')
fclose(fid);