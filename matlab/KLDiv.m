function dist=KLDiv(P,Q)
%  dist = KLDiv(P,Q) Kullback-Leibler divergence of two discrete probability
%  distributions
%  P and Q  are automatically normalised to have the sum of one on rows
% have the length of one at each 
% P =  n x nbins
% Q =  1 x nbins or n x nbins(one to one)
% dist = n x 1
% if size(P,2)~=size(Q,2)
%     error('the number of columns in P and Q should be the same');
% end
dist=0;
if sum(~isfinite(P(:))) + sum(~isfinite(Q(:)))
   error('the inputs contain non-finite values!') 
end
% normalizing the P and 
if size(Q,1)==1
    Q = Q ./sum(Q);
    P = P ./repmat(sum(P,2),[1 size(P,2)]);
    temp =  P.*log(P./repmat(Q,[size(P,1) 1]));
    temp(isnan(temp))=0;% resolving the case when P(i)==0
    dist = sum(temp,2);
    
elseif size(Q,1)==size(P,1)
    
    Q=double(Q);
    P=double(P);
    Q = Q./sum(Q,1);
    P = P./sum(P,1);
    temp =  P.*log(P./Q);
    temp(isnan(temp))=0.0;
    %temp(isnan(temp) & abs(Q-0)<0.0000001 )=0; % resolving the case when P(i)==0
    temp(isinf(temp))=10;
    %temp(isinf(temp))=0;
    dist = sum(abs(temp),1);
end