Projet : Calcul de la Matrice DH₅₋₀ et Trajectoire d’un Cube
🎯 Objectif du projet
Le but de ce projet est de :
1. Calculer la matrice de transformation homogène DH₅₋₀ d’un robot à 5 axes à partir du vecteur articulaire :
   Q = [Q₁, Q₂, Q₃, Q₄, Q₅]
2. Extraire les coordonnées cartésiennes (X, Y, Z) et les angles d’orientation (α, β, γ).
3. Générer une trajectoire composée de plusieurs configurations articulaires (50, 100, 200 ou 1000 points).
4. Calculer la matrice DH₅₋₀ pour chaque point et tracer :
   - l’évolution de la position (X, Y, Z) en 3D,
   - l’évolution des angles (α, β, γ) en 2D.
⚙️ Contenu du projet
- dh_matrix.py → Script Python de calcul de la matrice DH et de la trajectoire.
- rapport_DH.docx → Rapport complet en français, détaillant les étapes et le code.
- README.md → Description du projet.
🧠 Outils utilisés
• Python 3
• Bibliothèques : numpy, matplotlib, math
🚀 Exécution
python dh_matrix.py

📊 Résultats
- Affichage 3D de la trajectoire du point terminal.
- Graphiques 2D des angles d’orientation.
- Export possible en CSV ou PNG.
👨‍💻 Auteur
Projet réalisé par Abdel-Aziz Youssouf et Florian 
Étudiant à l’EPISEN – Formation ingénieur en informatique et cybersécurité appliquée à la santé.
