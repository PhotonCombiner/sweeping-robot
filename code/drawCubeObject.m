function drawCubeObject(cubeInfo,colorMatCube,pellucidity)
% ���������ϰ���ĺ���

if cubeInfo.exist
    for k1 = 1:size(cubeInfo.axisX,2)
        
        plotcube([cubeInfo.length(k1) cubeInfo.width(k1) cubeInfo.height(k1)],[cubeInfo.axisX(k1) cubeInfo.axisY(k1) cubeInfo.axisZ(k1)],pellucidity,colorMatCube);
    end
    
end

end

