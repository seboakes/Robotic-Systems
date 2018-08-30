

clf;
botSim.drawMap();


for k=1:topCandidates
    i= resultsFullSorted(k,1);
    particles(i).drawBot(3, 'black');
end



