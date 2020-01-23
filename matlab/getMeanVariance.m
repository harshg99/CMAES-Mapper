function [mean,variance]=getMeanVariance(vec)
mean=sum(vec)/size(vec,1);
variance=((vec-mean)'*(vec-mean))/size(vec,1);
end