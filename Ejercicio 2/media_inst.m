function MedPost = media_inst(NumPost,MapLabel)
MedPost = [];
for k = 1:NumPost
      m = min(find(MapLabel(:,3)==k));
      M = max(find(MapLabel(:,3)==k));
      Mpostes = MapLabel(m:M,1:2); % grupo de data por poste
      MedPst = [mean(Mpostes,1)]; % obtine la media de cada grupo
      MedPost = [MedPost;MedPst];
end
