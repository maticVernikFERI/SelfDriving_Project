# Treniranje Yolo modela
Da treniraš model moreš najprej naložit pytorch z Cuda (če hočeš uporabljat grafično), ali pa spremeniti device na ```device='cpu'```.
Glede na zmogljivost grafične in procesorja je treba prilagoditi tudi ```batch``` (v mojem primeru porabi 6-7GB vRAM-a) in ```workers``` (primerno za 16 jederni procesor) parametra.
Trenutno je za izhodišče uporabljen že najboljši model prejšnjega treniranje (```model = YOLO("best.pt")```), ki mora biti v isti mapi kot skripta, lahko se pa uporabi (```model = YOLO("yolo11n.pt")```), za default uteži yolo modela. Tak se lahko tudi spremeni velikost modela.
# Popravek anotacij
Pred začetkom treniranja moreš zagnati skripto nad mapama train in validate, da jima spremeniš polygone v bounding boxe. Če ne pride no opozorila pri nalaganju slik, treniranje bo vseeno uspešno, le da se te anotacije ne bodo upoštevale.