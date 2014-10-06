social-robotics-warriors
========================

A.A. 2014/2015, Mechatronic systems for rehabilitation - S1, UPMC


--------------------------------------------------------------------------------
  1  Uso di base di GIT
--------------------------------------------------------------------------------

Per informazioni aggiuntive, vedere queste guide:
http://www.git-scm.com/book/it
http://www.opentaps.org/docs/index.php/How_to_Use_Git:_a_Tutorial

Innanzitutto installiamo GIT:

    $ sudo apt-get install git

e configuriamolo con delle opzioni globali (ossia il vostro nome e la vostra
email):

    $ git config --global user.name "John Doe"
    $ git config --global user.email johndoe@example.com

--------------------------------------------------------------------------------
  1.1  Clonare un repository
--------------------------------------------------------------------------------

Clonare un repository GIT significa creare in locale (sul proprio PC) una copia
dei files contenuti nel repository in rete, così da poterli modificare senza
interferenze da parte di atri utenti. Nel nostro caso:

    $ git clone https://github.com/enripicci/social-robotics-warriors.git

che creerà una cartella chiamata "social-robotics-warriors" nella vostra
posizione attuale, con all'interno il repository clonato.
Ora possiamo lavorare su di esso:

    $ cd social-robotics-warriors

--------------------------------------------------------------------------------
  1.1  Informazioni sul repository
--------------------------------------------------------------------------------

Ora GIT ci permette di modificare i fiels in locale.
I files nel repository locale hanno 4 stati:
    * untracked   =   il file è presente ma non è incluso nel repository
    * modified    =   il file è incluso nel repository ma è stato modificato
    * staged      =   il file è stato marcato per il caricamento
    * committed   =   il file è aggiornato e caricato nel repository

Usare il comando

    $ git status

per visualizzare lo stato dei files nel repository.

    $ git diff
    $ git diff --staged

visualizzano informazioni molto dettagliate sui files modificati ed in stage,
rispettivamente.

--------------------------------------------------------------------------------
  1.2  Modificare i files
--------------------------------------------------------------------------------

Per aggiungere nuovi files, o per impostare un file modificato come staged,
il comando è:

    $ git add <nomefile> ...

    #=====================================================================#
    | NOTA: questo comando va dato per qualunque file modificato. I file  |
    | non messi in stage, difatti, non saranno inclusi nella procedura di |
    | commit (o saranno inclusi, ma con la versione precedente).          |
    | Se non si è sicuri, verificare con "git status".                    |
    |                                                                     |
    | vedi:  http://www.git-scm.com/book/it/Basi-di-Git-Salvare-le-       |
    |         modifiche-sul-repository                                    |
    #=====================================================================#

Una volta che tutti i file voluti sono impostati come stage, allora è il momento
di eseguire il commit.
Il commit corrisponde a salvare una nuova versione dei file, e viene effettuato
SOLAMENTE con i file in stage, NON con i semplici files modificati.
Il commit dovrebbe corrispondere ad un'avanzamento logico nella stesura del
programma, avanzamento che deve essere descritto da un commento.

    $ git commit -m "commento sul commit"
    $ git commit


    #=====================================================================#
    | NOTA: siccome la procedura di passaggio attraverso lo stage può     |
    | essere complicata e disagevole, il comando:                         |
    |                                                          .          |
    |   $ git commit -a -m "commento sul commit"                          |
    |   $ git commit -a                                                   |
    |                                                                     |
    | permette di includere nel commit pure tutti i file impostati come   |
    | modified, senza doverli prima marcare come stage.                   |
    #=====================================================================#

--------------------------------------------------------------------------------
  1.2  Rimuovere i files
--------------------------------------------------------------------------------

La rimozione dei files deve per forza passare attraverso l'area di stage.
Non è difatti sufficiente eliminarlo dal repository.
Il comando corrispondente è:

    $ git rm <nomefile>

che rimuoverà automaticamente il file dal repository, e ne terrà traccia deell'
eliminazione al prossimo commit.

    $ git rm --cached <nomefile>

è utilizzato per rimuovere un file dall'area di stage, senza rimuoverlo
fisicamente dal repository.


    


