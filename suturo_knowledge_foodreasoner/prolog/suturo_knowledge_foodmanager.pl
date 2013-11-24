%% SUTURO Knowledge - Food manager interface for python train script

:- module(suturo_knowledge_foodmanager,
    [
      get_food/1
    ]).

:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_edit')).
:- use_module(library('semweb/actionmodel')).

:- owl_parser:owl_parse('@LOCAL_PACKAGE_PATH@/owl/suturo_knowledge_hierarchy.owl', false, false, true).
:- owl_parser:owl_parse('@LOCAL_PACKAGE_PATH@/owl/orly.owl', false, false, true).

:- rdf_db:rdf_register_ns(hierarchy, 'http://www.suturo.de/ontology/hierarchy#', [keep(true)]).

% owl_has('http://www.suturo.de/m2/foodDB#Milka', rdf:type, hierarchy:'Edible').
% owl_has('http://www.suturo.de/m2/foodDB#Milka', rdf:type, hierarchy:'Box').
% owl_has('http://www.suturo.de/m2/foodDB#Milka', hierarchy:volumeOfObject, literal(type('http://www.w3.org/2001/XMLSchema#float', '6.0E-4'))).

%% get_food(Foodlist)
% 
% Description
%
% @param Foodlist describe me
%
get_food(Foodlist) :-
	true.


