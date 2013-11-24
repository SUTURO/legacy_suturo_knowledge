%% SUTURO Knowledge - Food query interface

:- module(suturo_knowledge_foodreasoner,
    [
      is_edible/2
    ]).

% TODO replace this with OWL ontology
edible(pringles).
edible(milka).
edible(testobj1). % FINDME for planning testing
volume(ball, 0.0024, 0.0001).
volume(pringles, 0.00085, 0.0001).
volume(tee, 0.00025, 0.00005).
volume(paket, 0.00515, 0.00025).
volume(milka, 0.0006, 0.00005).
volume(testobj1, 0.00339, 0.001). % FINDME for planning testing



%% is_edible(+ObjectList, -EdibleObject)
% 
% Queries the knowledge database for edibility of the given objects and returns a list of edible objects.
%
% @param ObjectList       List of percieved objects, containing id, label, object volume and object shape.
% @param EdibleObjectList List of edible objects.
%
is_edible(ObjectList, EdibleObjectList) :-
	findall(EdibleID, get_edible_objects(ObjectList, EdibleID), EdibleObjects),
	list_to_set(EdibleObjects, EdibleObjectList).

%$ Accessor predicate for perceived objects
perceived_object([ID, Label, Volume, Shape], ID, Label, Volume, Shape).

%% This parses float strings from LISP (1.23456789d-123) to prolog float
dfloat_to_float(StringDoubleFloat, Float) :-
	atomic_list_concat(List, 'd', StringDoubleFloat),
	atomic_list_concat(List, 'e', FloatString),
	atom_number(FloatString, Float).

%% Returns edible object ids
get_edible_objects(ObjectList, ID) :-
	member(SingleObject, ObjectList),
	perceived_object(SingleObject, ID, _, VolumeString, _),
	dfloat_to_float(VolumeString, Volume),
	edible(X),
	volume(X, Y, Z),
	Min is Y - Z,
	Max is Y + Z,
	Min =< Volume,
	Max >= Volume.