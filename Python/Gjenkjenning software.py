import cv2
import numpy as np
import argparse
import sys

def preprocess_frame(frame, blur_kernel=(99, 99), canny_lower=50, canny_upper=150, 
                     min_contour_area=100, clahe_cliplimit=2.0, clahe_tile=(8,8)):
    # Konverter bildet til gråtoner
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Bruk Gaussian blur for å redusere støy
    blurred = cv2.GaussianBlur(gray, blur_kernel, 99)
    
    # Forbedre kontrasten med CLAHE
    clahe = cv2.createCLAHE(clipLimit=clahe_cliplimit, tileGridSize=clahe_tile)
    equalized = clahe.apply(blurred)
    
    # Bruk Canny for kantdeteksjon
    edges = cv2.Canny(equalized, canny_lower, canny_upper)
    
    # Finn konturer i bildet
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filtrer ut små konturer basert på areal for å unngå støy
    filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]
    
    # Tegn de filtrerte konturene på en kopi av det originale bildet
    contour_frame = frame.copy()
    cv2.drawContours(contour_frame, filtered_contours, -1, (0, 255, 0), 2)
    
    return gray, blurred, equalized, edges, contour_frame, filtered_contours

def create_merged_view(images):
    merged_images = []
    for img in images:
        # Hvis bildet er i gråtone (to-dimensjonalt) → konverter til BGR
        if len(img.shape) == 2:
            img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            img_color = img
        merged_images.append(img_color)
    
    # Kombiner de første tre bildene horisontalt for rad 1
    row1 = cv2.hconcat([merged_images[0], merged_images[1], merged_images[2]])
    # Kombiner de neste tre bildene horisontalt for rad 2
    row2 = cv2.hconcat([merged_images[3], merged_images[4], merged_images[5]])
    # Kombiner radene vertikalt til et endelig bilde
    merged_view = cv2.vconcat([row1, row2])
    return merged_view

def run_camera(args):
    # Åpne standard kamera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Feil: Klarte ikke åpne kamera.")
        sys.exit(1)
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Feil: Klarte ikke lese bilde.")
                break
            
            # Prosesser bildet med parameterne
            gray, blurred, equalized, edges, contour_frame, contours = preprocess_frame(
                frame,
                blur_kernel=(args.blur_kernel, args.blur_kernel),
                canny_lower=args.canny_lower,
                canny_upper=args.canny_upper,
                min_contour_area=args.min_contour_area,
                clahe_cliplimit=args.clahe_cliplimit,
                clahe_tile=(args.clahe_tile, args.clahe_tile)
            )
            
            # Hvis merge view er aktivert, kombiner bildene til en enkelt visning
            if args.merge_view:
                # Bilder i rekkefølgen: original, gråtoner, blurred, CLAHE equalized, edges, konturer
                merged_view = create_merged_view([frame, gray, blurred, equalized, edges, contour_frame])
                cv2.imshow("Sammenslått visning", merged_view)
            else:
                cv2.imshow("Original Frame", frame)
                cv2.imshow("Gråtoner", gray)
                cv2.imshow("Blurred", blurred)
                cv2.imshow("CLAHE Equalized", equalized)
                cv2.imshow("Edges", edges)
                cv2.imshow("Konturer", contour_frame)
            
            # Trykk 'q' for å avslutte
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

def run_test_image(args, image_path):
    # Last inn bildet fra den angitte stien
    frame = cv2.imread(image_path)
    if frame is None:
        print("Feil: Klarte ikke laste inn bildet fra", image_path)
        sys.exit(1)
    
    # Prosesser bildet med de angitte parameterne
    gray, blurred, equalized, edges, contour_frame, contours = preprocess_frame(
        frame,
        blur_kernel=(args.blur_kernel, args.blur_kernel),
        canny_lower=args.canny_lower,
        canny_upper=args.canny_upper,
        min_contour_area=args.min_contour_area,
        clahe_cliplimit=args.clahe_cliplimit,
        clahe_tile=(args.clahe_tile, args.clahe_tile)
    )
    
    # Hvis merge view er aktivert, kombiner bildene til en enkelt visning
    if args.merge_view:
        merged_view = create_merged_view([frame, gray, blurred, equalized, edges, contour_frame])
        cv2.imshow("Sammenslått visning", merged_view)
    else:
        cv2.imshow("Original Frame", frame)
        cv2.imshow("Gråtoner", gray)
        cv2.imshow("Blurred", blurred)
        cv2.imshow("CLAHE Equalized", equalized)
        cv2.imshow("Edges", edges)
        cv2.imshow("Konturer", contour_frame)
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Forhåndsbehandle bildebilder ved hjelp av OpenCV for en robotsorteringsarm modul."
    )
    parser.add_argument('--test_image', type=str, help="Sti til testbilde filen.")
    parser.add_argument('--blur_kernel', type=int, default=5, 
                        help="Størrelse på Gaussian blur kernel (må være et oddetall).")
    parser.add_argument('--canny_lower', type=int, default=50, 
                        help="Nedre grense for Canny kantdeteksjon.")
    parser.add_argument('--canny_upper', type=int, default=150, 
                        help="Øvre grense for Canny kantdeteksjon.")
    parser.add_argument('--min_contour_area', type=float, default=100, 
                        help="Minimum areal for at en kontur skal vurderes.")
    parser.add_argument('--clahe_cliplimit', type=float, default=4.0, 
                        help="CLAHE clip-grense for kontrastforbedring.")
    parser.add_argument('--clahe_tile', type=int, default=8, 
                        help="Størrelse på CLAHE.")
    parser.add_argument('--merge_view', action='store_true', 
                        help="Aktiverer en kombinert visning.")
    args = parser.parse_args()
    
    # Hvis et testbilde er oppgitt, kjør det; ellers, bruk kameraet
    if args.test_image:
        run_test_image(args, args.test_image)
    else:
        run_camera(args)
