Installation électronique – Mise en place du setup
==================================================

Objectif
--------

Avant de commencer le projet (développement, documentation, ROS2, tests),
il est nécessaire de mettre en place un environnement de travail fonctionnel
autour de la Raspberry Pi.

Cette étape décrit uniquement le **branchement matériel de base** permettant
d’utiliser la Raspberry Pi comme un ordinateur classique.

Matériel nécessaire
-------------------

Pour démarrer la Raspberry Pi, le matériel suivant est requis :

- Une **Raspberry Pi** (avec carte micro-SD déjà installée)
- Un **écran**
- Un **clavier USB**
- Une **souris USB**
- Un **câble HDMI**
- Un **câble Ethernet**
- Une **alimentation pour la Raspberry Pi**

Types de câbles utilisés
------------------------

- **HDMI** :
  - Permet d’afficher l’interface graphique de la Raspberry Pi sur un écran.
  - Selon le modèle de Raspberry Pi, il peut s’agir d’un HDMI standard ou micro-HDMI.

- **Ethernet (RJ45)** :
  - Permet une connexion réseau stable.
  - Indispensable pour :
    - installer des paquets
    - cloner le dépôt GitHub
    - pousser les modifications (Git)
    - accéder à la documentation en ligne

- **USB (clavier et souris)** :
  - Le clavier et la souris sont branchés directement sur les ports USB de la Raspberry Pi.
  - Ils permettent une interaction locale sans accès distant.

- **Alimentation** :
  - La Raspberry Pi doit être alimentée avec une alimentation adaptée (USB-C ou micro-USB selon le modèle).
  - Une alimentation insuffisante peut provoquer des comportements instables.

Schéma de branchement (principe)
--------------------------------

1. Connecter l’écran à la Raspberry Pi à l’aide du câble HDMI.
2. Brancher le clavier et la souris sur les ports USB.
3. Connecter le câble Ethernet entre la Raspberry Pi et le réseau.
4. Brancher l’alimentation pour démarrer la Raspberry Pi.

Une fois ces branchements effectués, la Raspberry Pi démarre avec une interface
graphique et peut être utilisée comme un poste de travail classique
(terminal, éditeur de texte, navigateur).

Remarque
--------

Ce choix de setup local (écran + clavier + souris) permet de démarrer rapidement
le projet sans dépendre d’un accès distant (SSH). Une fois l’environnement configuré,
d’autres modes d’accès (SSH, VNC) peuvent être envisagés, mais ils ne sont pas nécessaires
pour la mise en place initiale.

