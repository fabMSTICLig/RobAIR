# Fichiers mécaniques

Les fichiers svg contiennent les pièces à découper au laser. Ils ont été édités avec InkScape. Ils ne sont pas utilisable en l'état afin de pouvoir les modifier. La plupart des pièces nécéssitent la manipulation suivante :
* Sélectionner toute la pièce
* Menu->Objet->Dégrouper
* Menu->Edition->Cloner->Délier le clone
* Selectionner les éléments bleus si présents puis Menu->Chemin->Union
* Sélectionner tous les éléments de la pièce sauf ceux avec un contour vert ou fond gris
* Menu->Chemin->Exclusion
* Mettre ensuite les paramètres pour votre découpeuse (Exemple : pas de fond, Contour : Couleur (R:255 G:0 B:0), Epaisseur:0.01in )

Les fichiers terminant par LC sont prêts à être découpé pour des planches de 400x600mm.


Les fichiers scad sont les fichiers sources OpenScad et les fichiers stl les fichiers correspondant à imprimer en 3D. 