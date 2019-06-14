function eul = quat2eul1(quat)

data_leng = size(quat,1);
eul = zeros(data_leng,3);

for k=1:data_leng
    eul_ = quat2EulerAngles(quat(k,1:4));
    eul(k,:) = eul_';
end

end