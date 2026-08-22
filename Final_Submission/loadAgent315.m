clear;

S = load("Agent315.mat");

agentObj = S.saved_agent;

assignin("base","agentObj",agentObj);

disp("Agent315 loaded successfully.");