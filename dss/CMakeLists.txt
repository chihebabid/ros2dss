# Définir la bibliothèque du sous-projet
project(dss_library)

# Ajouter les fichiers sources et en-têtes du sous-projet
file(GLOB_RECURSE SUBPROJECT_SOURCES src/*.cpp)
add_library(dss_library ${SUBPROJECT_SOURCES})

# Inclure les répertoires contenant les fichiers d'en-tête
target_include_directories(dss_library PUBLIC include)
