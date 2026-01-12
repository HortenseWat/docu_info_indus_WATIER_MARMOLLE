Tests
=====

Objectif des tests
------------------

Les tests ont pour objectif de **vérifier automatiquement** que certaines parties du projet
fonctionnent comme prévu, et de détecter rapidement les erreurs lors des modifications.

Dans ce projet, les tests servent principalement à :

- valider la logique Python indépendante de ROS2

- vérifier l’intégrité de certains fichiers (ex: URDF)

- éviter les régressions lors des commits

Stockage des tests 
------------------

Les tests que nous créons sont enregistrés dans un dossier /tests dans notre repository du projet. 
Chaque test et sous la forme d'un fichier python (extension ''.py'') et exécutés avec l’outil ``pytest``.

Types de tests mis en place
--------------------------

Deux types de tests ont été mis en place :

- **test unitaire** : vérifie une fonction ou un comportement précis
- **test d’intégration** : vérifie qu’un élément du projet est exploitable dans son ensemble


Test unitaire : validation du domaine
-------------------------------------

### Méthode utilisée

Un test unitaire ``test_domain.py`` a été créé pour vérifier que les entrées (ex: coordonnées, paramètres)
restent dans un domaine autorisé.

La logique est séparée dans un module Python indépendant de ROS2
(``pentograph_core``), ce qui permet de tester facilement sans dépendances lourdes.

Exemple de fonction testée :

.. code-block:: python

   def is_in_domain(x, y):
       return -0.2 <= x <= 0.2 and 0.0 <= y <= 0.3

Le test unitaire associé vérifie :

- un point valide → accepté

- un point hors domaine → refusé


Exemple de test :

.. code-block:: python

   def test_point_in_domain():
       assert is_in_domain(0.0, 0.1)

   def test_point_outside_domain():
       assert not is_in_domain(0.3, 0.1)

### Pourquoi ce test

Ce test permet de :

- détecter immédiatement une erreur logique

- éviter des entrées incohérentes dans le reste du projet

- garantir un comportement stable même après modification du code

Ce type de test est rapide, simple et très efficace.

Test unitaire : Vérification du Modèle géométrique inverse établi à l'aide d'une simulation CREO 
------------------------------------------------------------------------------------------------

On a aussi pu établir un test unitaire ``test_MGI.py`` permettant de comparer, pour une position (x, y) de l'effecteur du pantographe, des angles q1 et q4 calculés par notre modèle géométrique inverse. Pour ce faire, on récupère dans un tableau excel les valeurs de (x, y, q1 et q4) en fonction du temps, et l'on compare pour tous les (q1, q4) simulés, les (q1, q4) calculés. 

Ce test est très pertinent mais ne fonctionne pas bien (erreur dans notre MGI). 

Test d’intégration : validation URDF
------------------------------------

### Méthode utilisée

Un test d’intégration a été ajouté pour vérifier que le fichier URDF :

- existe bien dans le projet

- peut être lu et parsé sans erreur

Ce test ne vérifie pas la cinématique complète, mais garantit que le fichier
est **syntaxiquement valide** et exploitable par les outils ROS2.

Exemple de test :

.. code-block:: python

   def test_urdf_exists():
       assert os.path.exists("urdf/pantograph.urdf")

   def test_urdf_is_readable():
       with open("urdf/pantograph.urdf") as f:
           content = f.read()
           assert "<robot" in content

### Pourquoi ce test

Ce test permet de :

- détecter une suppression ou un déplacement accidentel du fichier URDF

- éviter des erreurs bloquantes lors de l’utilisation de ROS2 ou RViz

- sécuriser la structure du projet

Il s’agit d’un test d’intégration simple mais pertinent. Il n'a pas l'air entièrement fonctionnel cependant. 

Exécution des tests
-------------------

Les tests peuvent être lancés localement avec :

.. code-block:: bash

   pytest -q

Le paramètre ``-q`` permet d’obtenir une sortie concise
(indication claire succès / échec).

Pour lancer un test spécifique, on peut utiliser la commande suivante :

.. code-block:: bash

   pytest NOM-DU-DOSSIER-TESTS/nom-du-test
   
Dans notre cas par exemple, pour lancer le test unitaire : 

.. code-block:: bash

   pytest tests/test_domain.py

Intégration continue (CI)
-------------------------

Les tests sont exécutés automatiquement via **GitHub Actions** à chaque ``push``.

Principe :
- à chaque modification du dépôt

- les tests sont relancés automatiquement

- un échec empêche de valider une modification incorrecte

Cela permet de :

- détecter rapidement une régression

- garantir une base de code stable

- renforcer la qualité globale du projet

Lien avec le projet
-------------------

Les tests ne couvrent pas l’intégralité du projet (ROS2, RViz),
mais ciblent volontairement les parties critiques et facilement automatisables.

Cette approche permet d’avoir :

- des tests utiles

- un coût de maintenance faible

- une base saine pour faire évoluer le projet
