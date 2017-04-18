#-*- coding: UTF-8 -*-
import re;


filestr = r"./ir4stm32-all.pos";

tup_del = ('TST', 'LOGO');
dict_replace = {'C_0805':'0805',
                'C_B':'3528',
                'c_tant_B':'3528',
                'SMA':'DO-214AC',
                'SMA_Standard':'DO-214AC',
                'Crystal_SMD_5032_2Pads':'SMD-5032',
                'SO-4':'SOP-4_P2.54',
                'SO16':'SOIC-16',
                'LED-0805':'0805',
                'SOT23-EBC':'SOT-23',
                'SOT23':'SOT-23',
                'SOT223':'SOT-223',
                'R_1812':'1812',
                'TQFP_64':'LQFP-64',
                'DO-214AC':'LL-34',
                'crystal_FA238-TSX3225':'SMD-3225',
                '5032_2':'SMD-5032',
                'SO16':'SOIC-16',
                'SOIC-8_3.9X4.9MM_PITCH1.27MM':'SOIC-8'
                };

dict_replace_name = {
        'ULN2003':'ULN2003AFWG',
        '1N4148':'LL4148LL'
        };


f=open(filestr,"r");
txt=f.readlines();
f.close();


n = filestr.rfind('\\');
if n==-1:
        filename = filestr;
        filepath = "";
else:
        filename = filestr[n+1:];
        filepath = filestr[0:n+1];

n = filename.rfind('.');
if n==-1:
        filename_noext = filename;
else:
        filename_noext = filename[0:n];

# posfilename = filepath + "SMT_Coordinate_" + filename_noext + ".csv";
# bomfilename = filepath + "SMT_BOM_" + filename_noext + ".csv";
posfilename = "./ir4stm32-all.pos"
bomfilename = "./ir4stm32.csv"

goods = dict();
pos = list();


for i in txt:
        if re.match('#', i):
                continue;
        l = re.split('[ \n\r]+', i);
        if l[1] in tup_del:
                continue;
        if l[1] in dict_replace_name.keys():
                l[1] = dict_replace_name[l[1]];
        if l[2] in dict_replace.keys():
                l[2] = dict_replace[l[2]];
        x = [l[0], l[2], l[3]+'mm', l[4]+'mm', 'T', l[5]];
        #print(x);
        pos.append(x);
        #print(l);
        l1 = str(l[1]).upper();
        l2 = str(l[2]).upper();
        x = tuple((l1,l2));
        if x not in goods.keys():
                goods[x] = list();
        goods[x].append(l[0]);

#write pos file
f = open(posfilename, 'w');
f.write('Designator,Footprint,Mid X,Mid Y,Layer,Rotation\n');
for i in pos:
        s = str(i[0])+','+\
                str(i[1])+','+\
                str(i[2])+','+\
                str(i[3])+','+\
                str(i[4])+','+\
                str(i[5])+'\n';
        f.write(s);
f.close();

#write bom file
f = open(bomfilename, 'w');
f.write('Comment,Designator,Footprint\n');
for i in goods:
        ss = i[0]+",";
        comp = goods[i];
        if len(comp)>1:
                ss = ss + '"' + comp[0];
                for x in comp[1:]:
                        ss = ss + "," + x;
                ss = ss + '",';
        else:
                ss = ss + comp[0] + ",";
        ss = ss + i[1]+"\n";
        f.write(ss);

f.close();
