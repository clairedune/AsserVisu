deff('y=f(x)','y=exp(-x.^2)')//une bosse
x=[-10:0.05:10];
rect=[-10,-0.2,10,1];//taille de la fenêtre
plot2d(0,0,rect=rect,frameflag=3)//initialisation de la fenêtre graphique
winnum=winsid();//numéro des fenêtres graphiques
toolbar(winnum($),'off');//enlever la barre d'outils de la dernière fenêtre
xset("pixmap",1)//ouvrir le buffer graphique
driver("Rec") //driver pour sauver les images
k=0//numéro d'image
for t=-10:0.05:10//boucle d'animation
k=k+1;//numéro nouvelle image
y=f(x-t);//la bosse qui glisse
xbasc()//effacement de la fenêtre
clear_pixmap()//et buffer
plot2d(x,y,rect=rect,axesflag=5,frameflag=1)//nouvelle image dans le buffer
show_pixmap()//affichage du contenu du buffer
if pmodulo(k,10)==0 then //je sauve une image sur 10
nom_image='image_'+string(1000+k)+'.gif';
xs2gif(winnum($),nom_image);//on sauve dans le répertoire courant
end
end
xset("pixmap",0)//fermer le buffer graphique 
