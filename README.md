# 1 Prise en main de la carte STM32 et des capteurs
 ### 1.3 Prise en main de la carte (allumer deux LEDS)
   
 ### 1.4 Premiers pas avec le bus I2C
   * **Quand a été inventé le bus I2C et quelle est sa version actuelle?**
     *Le bus I²C (Inter-Integrated Circuit) a été inventé par Philips (aujourd'hui NXP Semiconductors) en 1982. La dernière version du standard I²C est la version 7.0, publiée en 2021.*
   * **Combien de lignes nécessite le bus I2C? Quelle est leur fonction?**
     ***SCL (Serial Clock Line) :** Cette ligne transporte le signal d'horloge généré par le maître pour synchroniser la communication.*
     ***SDA (Serial Data Line) :** Cette ligne transporte les données entre le maître et les périphériques esclaves.*
   * **Le bus I2C est un bus série, synchrone, bidirectionnel et half-duplex. Donner la définition des ces 4 éléments.**
     
     ***Série :** Les données sont transmises bit par bit sur une seule ligne de données (SDA), ce qui limite le nombre de fils nécessaires par rapport aux transmissions parallèles.*
     
***Synchrone :** La transmission est synchronisée par un signal d'horloge (SCL) fourni par le maître, permettant une communication coordonnée entre les périphériques.*

***Bidirectionnel :** Le bus permet aux données de circuler dans les deux sens, du maître vers l’esclave et inversement, selon les besoins.*

***Half-duplex :** La communication se fait dans un seul sens à la fois. Cela signifie qu’à un moment donné, le bus I²C peut soit émettre, soit recevoir, mais pas les deux simultanément.*

   * **Sur combien de bits est « classiquement » codée l’adresse d’un périphérique sur un bus I2C?**
     
   * **Combien de périphériques peuvent être alors connectés au même bus?**
     
   * **Quelle est la vitesse de transmission possible des octets sur un bus I2C?**
     

 
 
 
 

 
