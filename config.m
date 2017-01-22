function [A,B,T] = config(D)

if nnz(D(:,1)) == 2
    T = 'dd';
    L = D(find(D));
    A = L(1);
    B = L(2);
elseif nnz(D(:,2)) == 2
    T = 'tt';
    L = D(find(D));
    A = L(1);
    B = L(2);
elseif nnz(D) == 2 && D(1,1) > 0 
    L = D(find(D));  
        T = 'dt';
        A = L(1);
        B = L(2);
else
        T = 'td';
        L = D(find(D));
        A = L(2);
        B = L(1);    
end

disp(D)

end
