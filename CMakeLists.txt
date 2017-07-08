cmake_minimum_required(VERSION 3.0)

add_definitions(-DAMPLMODELFOLDERPATH="/home/henning/Documents/Masters_Thesis/AMPE/source/MILP_Planner/build/amplModel/")

add_definitions(-DUAVPARAMLOCATION="/home/henning/Documents/Masters_Thesis/AMPE/source/MPC/source/")



ADD_SUBDIRECTORY( source )
