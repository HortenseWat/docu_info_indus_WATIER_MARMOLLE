Repository (Git + GitHub)
========================

Objectif
--------

L’objectif est d’avoir un dépôt Git propre, versionné et synchronisé avec GitHub.
Cela permet :

- de garder l’historique des modifications (qui a fait quoi, quand, pourquoi)
- de travailler à plusieurs sans se marcher dessus
- de déclencher automatiquement des actions (ex: déploiement de la doc, tests) à chaque ``push``

Configuration Git (identité)
----------------------------

Avant de faire des commits, on configure l’identité Git (nom + email).
Cela sert à associer chaque commit à une personne.

.. code-block:: bash

   git config --global user.name "HortenseWat"
   git config --global user.email "hortense.watier@insa-strasbourg.fr"

Remarque : on a d’abord nettoyé une ancienne configuration (mauvais utilisateur) :

.. code-block:: bash

   git config --global --unset user.name
   git config --global --unset user.email

Clé SSH (authentification GitHub)
---------------------------------

Quand on veut pousser sur GitHub, il faut s’authentifier. GitHub n’autorise plus l’authentification
par mot de passe en HTTPS pour les opérations Git (push/pull). On a donc utilisé SSH.

1) Génération d’une clé SSH
^^^^^^^^^^^^^^^^^^^^^^^^^^^

On supprime d’anciennes clés (si besoin), puis on génère une clé ED25519 :

.. code-block:: bash

   rm -rf ~/.ssh/id_ed25519*
   ssh-keygen -t ed25519 -C "hortense.watier@insa-strasbourg.fr"

À la fin, on obtient :

- la clé privée : ``~/.ssh/id_ed25519`` (à garder secrète)
- la clé publique : ``~/.ssh/id_ed25519.pub`` (à copier sur GitHub)

2) Récupérer la clé publique
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   cat ~/.ssh/id_ed25519.pub

La clé publique est ensuite ajoutée dans GitHub :
Settings → SSH and GPG keys → New SSH key.

3) Tester la connexion SSH à GitHub
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ssh -T git@github.com

Lors du premier essai, GitHub demande de confirmer l’empreinte (normal).
On a validé puis GitHub confirme l’authentification :

- *"You've successfully authenticated, but GitHub does not provide shell access."*

Clonage du dépôt
----------------

Une fois le dépôt GitHub créé, on le clone sur la Raspberry Pi :

.. code-block:: bash

   git clone https://github.com/HortenseWat/docu_info_indus_WATIER_MARMOLLE.git
   cd docu_info_indus_WATIER_MARMOLLE

Rappel : workflow Git minimal
----------------------------

.. code-block:: bash

   # 1) voir ce qui a changé
   git status

   # 2) ajouter les fichiers modifiés
   git add .

   # 3) créer le commit
   git commit -m "message clair"

   # 4) envoyer sur GitHub
   git push



Premiers commits
----------------

1) Modifier un fichier (ex: README)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

On a modifié ``README.md`` avec un éditeur (ici ``nano``) :

.. code-block:: bash

   nano README.md

2) Ajouter les modifications (staging)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

On doit ajouter les fichiers modifiés à l’index avec ``git add``.

Erreur rencontrée :
- On a essayé ``git add all`` → Git a compris "all" comme un nom de fichier, donc erreur.

.. code-block:: bash

   git add all
   # fatal: pathspec 'all' did not match any files

Solution :
- Utiliser ``git add .`` pour ajouter toutes les modifications du dossier courant.

.. code-block:: bash

   git add .

3) Commit
^^^^^^^^^

Le commit enregistre un état du projet + un message explicite.

.. code-block:: bash

   git commit -m "Premier commit : test et modif readme"

Push vers GitHub (et problème HTTPS)
------------------------------------

On a ensuite voulu pousser sur GitHub en HTTPS :

.. code-block:: bash

   git push -u origin main

Erreur rencontrée :
- GitHub refuse le mot de passe : *"Password authentication is not supported for Git operations."*
C’est normal : GitHub impose SSH ou token.

Solution : passer le remote en SSH
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

On a remplacé l’URL du remote ``origin`` par l’URL SSH :

.. code-block:: bash

   git remote set-url origin git@github.com:HortenseWat/docu_info_indus_WATIER_MARMOLLE.git
   git remote -v

Puis le push fonctionne :

.. code-block:: bash

   git push -u origin main

Bonnes pratiques utilisées
--------------------------

- Commits réguliers et messages clairs (ex: *"docs: ..."*, *"test: ..."*)
- Remote en SSH (évite la gestion de tokens)
- Répertoire propre : pas de fichiers générés (build, caches, etc.) versionnés



