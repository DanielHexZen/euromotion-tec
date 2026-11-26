function A = nav_sl_unlog(A)
%NAV_SL_UNLOG  To-Workspace 'Array' log -> [N x n] matrix (handles n x 1 x N logs).
if ndims(A) == 3
    A = reshape(A, size(A,1)*size(A,2), size(A,3)).';
end
end
