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
  1.1  Modificare i files
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






