//
//  PitchLadder.swift
//  PrimaryFlightDisplay
//
//  Created by Michael Koukoullis on 5/01/2016.
//  Copyright © 2016 Michael Koukoullis. All rights reserved.
//

import SpriteKit

class PitchLadder: SKNode {
    
    let sceneSize: CGSize
    fileprivate let cropNode = SKCropNode()
    fileprivate let maskNode: SKSpriteNode
    
    init(sceneSize: CGSize, style: PitchLadderStyleType) {
        self.sceneSize = sceneSize
        let maskSize = CGSize(
            width:  CGFloat(style.majorLineWidth) * 3.0,
            height: sceneSize.pointsPerDegree * (CGFloat(style.magnitudeDisplayDegree) + 4))
        self.maskNode = SKSpriteNode(color: SKColor.black, size: maskSize)
        super.init()
        
        let builder = PitchLineBuilder(style: style)
        let degreeValues = Array(stride(from: 5, to: 91, by: 5))
        
        let skyPitchLines = degreeValues.map { degree in
            return (degree, (degree % 10 == 0) ? PitchLineType.major : PitchLineType.minor)
        }
        let pitchLines = skyPitchLines + skyPitchLines.map { ($0.0 * -1, $0.1) }
        for (degree, type) in pitchLines {
            cropNode.addChild(builder.pitchLine(sceneSize: sceneSize, degree: degree, type: type))
        }
        for (degree, type) in pitchLines.filter({ $1 == .major }) {
            cropNode.addChild(builder.leftPitchLineLabel(sceneSize: sceneSize, degree: degree, type: type))
            cropNode.addChild(builder.rightPitchLineLabel(sceneSize: sceneSize, degree: degree, type: type))
        }
        
        cropNode.maskNode = maskNode
        addChild(cropNode)
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}

extension PitchLadder: AttitudeSettable {
    
    func setAttitude(_ attitude: AttitudeType) {
        cropNode.run(attitude.pitchAction(sceneSize: sceneSize))
        maskNode.run(attitude.pitchReverseAction(sceneSize: sceneSize))
        run(attitude.rollAction())
    }
}

private enum PitchLineType {
    case major
    case minor
}

private struct PitchLineBuilder {
    
    let style: PitchLadderStyleType
    
    func pitchLine(sceneSize: CGSize, degree: Int, type: PitchLineType) -> SKShapeNode {
        
        let halfWidth = halfWidthForPitchLineType(type)
        let path = CGMutablePath()
        
        path.move(to: CGPoint(x: -halfWidth, y: 2))
        path.addLine(to: CGPoint(x: halfWidth, y: 2))
        path.addLine(to: CGPoint(x: halfWidth, y: -2))
        path.addLine(to: CGPoint(x: -halfWidth, y: -2))
        
        path.closeSubpath()
        
        var transform = CGAffineTransform(translationX: 0, y: CGFloat(degree) * sceneSize.pointsPerDegree)
        let transformedPath = withUnsafeMutablePointer(to: &transform) {
            path.mutableCopy(using: $0)
        }

        let line = SKShapeNode(path: transformedPath!)
        line.fillColor = style.fillColor
        line.strokeColor = style.strokeColor
        return line
    }
    
    func leftPitchLineLabel(sceneSize: CGSize, degree: Int, type: PitchLineType) -> SKLabelNode {
        let label = pitchLineLabel(sceneSize: sceneSize, degree: degree, type: type)
        label.horizontalAlignmentMode = .right
        label.position.x = -halfWidthForPitchLineType(type) - CGFloat(style.markerTextOffset)
        return label
    }

    func rightPitchLineLabel(sceneSize: CGSize, degree: Int, type: PitchLineType) -> SKLabelNode {
        let label = pitchLineLabel(sceneSize: sceneSize, degree: degree, type: type)
        label.horizontalAlignmentMode = .left
        label.position.x = halfWidthForPitchLineType(type) + CGFloat(style.markerTextOffset)
        return label
    }

    fileprivate func pitchLineLabel(sceneSize: CGSize, degree: Int, type: PitchLineType) -> SKLabelNode {
        let label = SKLabelNode(text: "\(degree)")
        label.fontName = style.font.family
        label.fontSize = style.font.size
        label.fontColor = style.textColor
        label.verticalAlignmentMode = .center
        label.position.y = CGFloat(degree) * sceneSize.pointsPerDegree
        return label
    }
    
    fileprivate func widthForPitchLineType(_ type: PitchLineType) -> CGFloat {
        switch type {
        case .major: return CGFloat(style.majorLineWidth)
        case .minor: return CGFloat(style.minorLineWidth)
        }
    }
    
    fileprivate func halfWidthForPitchLineType(_ type: PitchLineType) -> CGFloat {
        return widthForPitchLineType(type) / 2
    }
}
